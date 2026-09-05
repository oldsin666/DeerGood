/*
 * line_patrol.c —— 黑胶带循迹巡航（Y 路口随机选路 / 干字型终点停车 / 死路掉头）
 *
 * 场地约定：路径 = 黑色胶带；路径两侧/外围 = 白色胶带（作为边界）。
 * 传感器：车底 TCRT5000 ×2（IO13=左，IO14=右），经 LM393 比较器输出数字量。
 *
 * 功能：
 *   1. 沿黑线走；压到白（偏离）自动差速修正回线。
 *   2. 遇到 Y 路口（黑区变宽，两探头同时压黑）→ 随机选左边或右边一条。
 *   3. 3 秒内再次遇到 Y 路口 = 说明是「干」字型路口（两个 Y 相连）→ 判定终点 → 自动停车。
 *   4. 长时间黑和白都检测不到（黑线消失）= 走进死路 → 原地掉头返回。
 *
 * ---------------------------------------------------------------------------
 * 【重要】探头布局（LP_STRADDLE_LAYOUT）必须与实际安装一致，否则转向方向会反：
 *
 *   LP_LINE_INSIDE = 1（默认，对应你的描述）：
 *       两个探头间距 < 黑线宽，正常行驶时两探头都在黑线上。
 *         11(都黑) = 走得好   → 直行
 *         10(左黑右白) = 车偏右 → 左转修正
 *         01(左白右黑) = 车偏左 → 右转修正
 *         00(都白)   = 完全脱离黑线 → 持续超过阈值判定死路
 *       路口：黑区变宽，11 会持续较长时间 → 用「11 持续时长」判定 Y 路口。
 *
 *   LP_LINE_INSIDE = 0（跨线布局）：
 *       两探头间距 > 黑线宽，正常行驶时黑线从两探头中间穿过。
 *         00(都不黑) = 线在中间 → 直行
 *         10(左黑)   = 线偏左   → 左转修正
 *         01(右黑)   = 线偏右   → 右转修正
 *         11(都黑)   = 黑区变宽 → Y 路口
 *       死路：长时间「看不到黑线」→ 持续 00 超过阈值。
 * ---------------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdint.h>
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"

/* ================== 实车标定区（这些值必须上车调，别指望一次就对） ================== */
#define LP_TICK_MS              20      // 控制周期(ms)，越小越灵敏但越耗 CPU

#define LP_LINE_INSIDE          1       // 1=两探头都在黑线内(默认) / 0=两探头跨在黑线两侧
#define LP_ACTIVE_HIGH          1       // 1=输出高电平表示压到黑（LM393 常见）；实测反了改 0

#define LP_BASE_SPEED           55      // 直行油门(0~150)
#define LP_FIX_SLOW             18      // 修正时「内侧轮」速度（越小转得越急）
#define LP_FIX_FAST             62      // 修正时「外侧轮」速度

#define LP_JUNCTION_HOLD_MS     600     // 两探头同时压黑持续多久认定为 Y 路口。
                                        // 调大=不容易误判(长直道安全)但可能漏检；调小=灵敏但直道可能误触发。
#define LP_JUNCTION_REENTRY_MS  3000    // 3 秒内再次遇到 Y 路口 = 干字型 → 终点停车
#define LP_JUNCTION_COOLDOWN_MS 400     // 脱离路口后的冷却时间，防止同一个路口被重复触发

#define LP_TURN_SPEED           45      // Y 路口转向速度
#define LP_TURN_MIN_MS          300     // 转向至少保持这么久（先转离原路，否则会立刻又看到原路而停下）
#define LP_TURN_MAX_MS          1500    // 转向最长时长（超时强制退出，防止转不停）

#define LP_LOST_TIMEOUT_MS      1000    // 黑和白都检测不到持续多久判定为死路
#define LP_UTURN_SPEED          50      // 掉头速度
#define LP_UTURN_MIN_MS         700     // 掉头至少转这么久（约 180°，需按你的车速/轮胎实车调）
#define LP_UTURN_MAX_MS         2200    // 掉头最长时长

#define LP_IO_LEFT              WIFI_IOT_IO_NAME_GPIO_13
#define LP_IO_RIGHT             WIFI_IOT_IO_NAME_GPIO_14
#define LP_UART_IDX             WIFI_IOT_UART_IDX_2

/* ================== 底层：电机 / 传感器 / 随机数 ================== */

// 双电机控制：与 main.c 的 stm32_motor_control 同一协议
// 6 字节帧：[0xFC][左方向][左速度][右方向][右速度][0xFD]，方向 0=正转 1=反转，速度 0~150
static void Lp_Motors(int left, int right)
{
    unsigned char buf[6];
    unsigned char lDir = 0, rDir = 0;

    if (left < 0)  { lDir = 1; left  = -left;  }
    if (right < 0) { rDir = 1; right = -right; }
    if (left  > 150) left  = 150;
    if (right > 150) right = 150;

    buf[0] = 0xFC;
    buf[1] = lDir;
    buf[2] = (unsigned char)left;
    buf[3] = rDir;
    buf[4] = (unsigned char)right;
    buf[5] = 0xFD;

    UartWrite(LP_UART_IDX, buf, sizeof(buf));
}

static void Lp_Stop(void)   { Lp_Motors(0, 0); }
static void Lp_Forward(void){ Lp_Motors(LP_BASE_SPEED, LP_BASE_SPEED); }

// 差速修正：dir=0 左转（右轮快左轮慢），dir=1 右转
static void Lp_Fix(int dir)
{
    if (dir == 0) Lp_Motors(LP_FIX_SLOW, LP_FIX_FAST);   // 左转
    else          Lp_Motors(LP_FIX_FAST, LP_FIX_SLOW);   // 右转
}

// 原地转向：dir=0 左转，dir=1 右转（两轮反向，转得比差速更急，用于路口/掉头）
static void Lp_Spin(int dir, int speed)
{
    if (dir == 0) Lp_Motors(-speed, speed);
    else          Lp_Motors(speed, -speed);
}

// 读两个 TCRT：1=压到黑，0=非黑（白/地面）
static void Lp_ReadIR(uint8_t *l, uint8_t *r)
{
    WifiIotGpioValue vl = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue vr = WIFI_IOT_GPIO_VALUE0;

    GpioGetInputVal(LP_IO_LEFT,  &vl);
    GpioGetInputVal(LP_IO_RIGHT, &vr);

    *l = (vl == WIFI_IOT_GPIO_VALUE1) ? 1u : 0u;
    *r = (vr == WIFI_IOT_GPIO_VALUE1) ? 1u : 0u;

#if !LP_ACTIVE_HIGH
    *l = (uint8_t)(*l ^ 1u);
    *r = (uint8_t)(*r ^ 1u);
#endif
}

// 简易随机数（避免用 rand 的全局状态）：用于 Y 路口随机选路
static unsigned int lp_rand_state;
static unsigned int Lp_Rand(void)
{
    lp_rand_state = lp_rand_state * 1103515245u + 12345u;
    return (lp_rand_state >> 16) & 0x7FFFu;
}

/* ================== 状态机 ================== */

typedef enum {
    LP_FOLLOW = 0,      // 循迹中
    LP_JUNCTION,        // Y 路口：随机选一条
    LP_UTURN,           // 死路掉头
    LP_FINISH           // 终点停车（不再动作）
} LpState;

void LinePatrol_Thread(void *arg)
{
    (void)arg;

    // 探头引脚（main.c 的 TCRT_Init 也会设一次，重复设置无害）
    IoSetFunc(LP_IO_LEFT,  WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(LP_IO_RIGHT, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(LP_IO_LEFT,  WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(LP_IO_RIGHT, WIFI_IOT_GPIO_DIR_IN);

    lp_rand_state = 20260905u;   // 固定种子便于复现；想每次不同可换成传感器读数

    LpState state = LP_FOLLOW;
    unsigned int nowMs        = 0;    // 运行时间(ms)
    unsigned int bothBlackMs  = 0;    // “两探头同时压黑”已持续时长 —— 判定 Y 路口
    unsigned int lostMs       = 0;    // “黑和白都检测不到”已持续时长 —— 判定死路
    unsigned int actionMs     = 0;    // 当前转向/掉头动作已执行时长
    unsigned int cooldownMs   = 0;    // 路口冷却计时
    unsigned int lastJuncMs   = 0;    // 上一次路口发生的时刻
    int  haveLastJunc  = 0;           // 是否已经遇到过路口（首次路口不能判干型）
    int  turnDir       = 0;           // 0=左 1=右

    printf("[LINE] patrol thread start (layout=%s)\r\n",
           LP_LINE_INSIDE ? "line-inside" : "straddle");

    while (1) {
        uint8_t l = 0, r = 0;
        Lp_ReadIR(&l, &r);
        nowMs += LP_TICK_MS;

        if (state == LP_FINISH) {          // 终点：保持停车
            Lp_Stop();
            osDelay(LP_TICK_MS);
            continue;
        }

        if (cooldownMs > 0) {
            cooldownMs = (cooldownMs > LP_TICK_MS) ? (cooldownMs - LP_TICK_MS) : 0;
        }

        /* ---------- 转向 / 掉头动作中：先转够最小时长，再等重新捕获黑线 ---------- */
        if (state == LP_JUNCTION || state == LP_UTURN) {
            actionMs += LP_TICK_MS;

            int minMs  = (state == LP_JUNCTION) ? LP_TURN_MIN_MS  : LP_UTURN_MIN_MS;
            int maxMs  = (state == LP_JUNCTION) ? LP_TURN_MAX_MS  : LP_UTURN_MAX_MS;
            int speed  = (state == LP_JUNCTION) ? LP_TURN_SPEED   : LP_UTURN_SPEED;

            if (actionMs <= (unsigned int)minMs) {
                Lp_Spin(turnDir, speed);                 // 先转离原路
            } else if ((l || r) || actionMs >= (unsigned int)maxMs) {
                // 重新看到黑线（或超时兜底）→ 回到循迹
                printf("[LINE] turn done: dir=%s, %s, %ums\r\n",
                       turnDir ? "right" : "left",
                       (l || r) ? "line re-captured" : "timeout",
                       (unsigned)actionMs);
                state = LP_FOLLOW;
                actionMs = 0;
                bothBlackMs = 0;
                lostMs = 0;
                cooldownMs = LP_JUNCTION_COOLDOWN_MS;
            } else {
                Lp_Spin(turnDir, speed);                 // 继续转，等待捕获
            }

            osDelay(LP_TICK_MS);
            continue;
        }

        /* ---------------------------- 循迹中 ---------------------------- */

        // 统计两类“持续时间”
        if (l && r) {                 // 两探头都压黑
            bothBlackMs += LP_TICK_MS;
            lostMs = 0;
        } else if (!l && !r) {        // 黑和白都检测不到
            lostMs += LP_TICK_MS;
            bothBlackMs = 0;
        } else {                      // 单边压黑：正常修正中
            lostMs = 0;
            bothBlackMs = 0;
        }

        // （1）死路判定：长时间黑和白都检测不到 → 原地掉头
        if (lostMs >= LP_LOST_TIMEOUT_MS) {
            printf("[LINE] LOST line (%ums) -> U-turn (dead end)\r\n", (unsigned)lostMs);
            state = LP_UTURN;
            actionMs = 0;
            lostMs = 0;
            turnDir = (int)(Lp_Rand() & 1u);
            haveLastJunc = 0;       // 掉头后清空路口记录，避免返回时误判“干型终点”
            lastJuncMs = 0;
            osDelay(LP_TICK_MS);
            continue;
        }

        // （2）Y 路口判定：两探头同时压黑持续足够久（黑区变宽）
        if (bothBlackMs >= LP_JUNCTION_HOLD_MS && cooldownMs == 0) {
            if (haveLastJunc && (nowMs - lastJuncMs) <= LP_JUNCTION_REENTRY_MS) {
                // 短时间内第二次遇到 Y → 干字型路口 → 终点
                printf("[LINE] DRY junction (2nd Y within %ums) -> FINISH, stop\r\n",
                       (unsigned)(nowMs - lastJuncMs));
                Lp_Stop();
                state = LP_FINISH;
                osDelay(LP_TICK_MS);
                continue;
            }

            turnDir = (int)(Lp_Rand() & 1u);
            printf("[LINE] Y junction -> random pick %s\r\n", turnDir ? "right" : "left");
            lastJuncMs = nowMs;
            haveLastJunc = 1;
            state = LP_JUNCTION;
            actionMs = 0;
            osDelay(LP_TICK_MS);
            continue;
        }

        // （3）普通循迹：按布局解释 (l,r) 并修正
#if LP_LINE_INSIDE
        if (l && r) {
            Lp_Forward();                       // 都在黑线上：走得好
        } else if (l && !r) {
            Lp_Fix(0);                          // 左黑右白：车偏右 → 左转修正
        } else if (!l && r) {
            Lp_Fix(1);                          // 左白右黑：车偏左 → 右转修正
        } else {
            Lp_Forward();                       // 都白：短暂脱线，先直行找线（超时才判死路）
        }
#else
        if (l && r) {
            Lp_Forward();                       // 黑区变宽：等持续时长后判为路口，这里先直行
        } else if (l && !r) {
            Lp_Fix(0);                          // 左黑：线偏左 → 左转修正
        } else if (!l && r) {
            Lp_Fix(1);                          // 右黑：线偏右 → 右转修正
        } else {
            Lp_Forward();                       // 线在两探头中间：直行
        }
#endif

        osDelay(LP_TICK_MS);
    }
}
