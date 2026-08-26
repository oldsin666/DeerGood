/*
 * 先锋号鸿蒙智能小车 - 电机 + 车顶舵机演示
 * 功能：
 *   1) 车轮前进/停止：L9110S 电机，Hi3861 通过 UART2 发协议给 STM32 控制板
 *   2) 车顶元件转动：SG90 舵机（接 GPIO2）左右转动、回中
 */
#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "hi_io.h"
#include "hi_time.h"
#include "hi_uart.h"

/* SG90 舵机接 GPIO2（车顶元件） */
#define GPIO2 2

/* UART2 串口参数：与 STM32 控制板通信，波特率 115200 */
static WifiIotUartAttribute g_uart_attr = {
    .baudRate = 115200,
    .dataBits = 8,
    .stopBits = 1,
    .parity = 0,
};

/* ==================== 电机控制（UART2 -> STM32 -> L9110S） ==================== */

/*
 * 发送给 STM32 的电机控制协议：
 * 帧头 0xFC + 左轮方向 + 左轮速度 + 右轮方向 + 右轮速度 + 帧尾 0xFD
 * motorA / motorB：左右轮速度（rad/s 的一百倍），范围 -150 ~ 150
 */
static void stm32motor_control(int motorA, int motorB)
{
    unsigned char uart_sendbuf[6];
    unsigned char A_dir = 0;
    unsigned char B_dir = 0;

    /* 确认旋转方向：正转 0，反转 1 */
    if (motorA < 0) {
        A_dir = 1;
        motorA = -motorA;
    } else {
        A_dir = 0;
    }
    if (motorB < 0) {
        B_dir = 1;
        motorB = -motorB;
    } else {
        B_dir = 0;
    }

    /* 限制幅度 ±150 */
    if (motorA > 150) {
        motorA = 150;
    }
    if (motorB > 150) {
        motorB = 150;
    }

    uart_sendbuf[0] = 0xFC;   /* 帧头 */
    uart_sendbuf[1] = A_dir;  /* 左轮方向 0正转 1反转 */
    uart_sendbuf[2] = motorA; /* 左轮速度 */
    uart_sendbuf[3] = B_dir;  /* 右轮方向 */
    uart_sendbuf[4] = motorB; /* 右轮速度 */
    uart_sendbuf[5] = 0xFD;   /* 帧尾 */

    UartWrite(WIFI_IOT_UART_IDX_2, uart_sendbuf, 6);
}

static void car_forward(void)   /* 小车前进 */
{
    stm32motor_control(100, 100);
}

static void car_stop(void)      /* 小车停止 */
{
    stm32motor_control(0, 0);
}

/* ==================== 舵机控制（GPIO2 -> SG90，车顶元件） ==================== */

/* 输出 20ms 周期脉冲：duty 微秒高电平，20000-duty 微秒低电平 */
static void set_angle(unsigned int duty)
{
    GpioSetDir(GPIO2, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(GPIO2, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);
    GpioSetOutputVal(GPIO2, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}

static void engine_turn_left(void)   /* 舵机左转 */
{
    for (int i = 0; i < 10; i++) {
        set_angle(2200);
    }
}

static void engine_turn_right(void)  /* 舵机右转 */
{
    for (int i = 0; i < 10; i++) {
        set_angle(1100);
    }
}

static void regress_middle(void)     /* 舵机回中 */
{
    for (int i = 0; i < 10; i++) {
        set_angle(1650);
    }
}

/* ==================== 初始化 ==================== */
static void Car_Init(void)
{
    GpioInit();  /* GPIO 功能初始化 */

    /* 舵机 GPIO2 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_GPIO_DIR_OUT);

    /* UART2：与 STM32 通信（GPIO11 TXD / GPIO12 RXD） */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    UartInit(WIFI_IOT_UART_IDX_2, &g_uart_attr, NULL);
}

/* ==================== 主程序 ==================== */
static void CarMain(void)
{
    Car_Init();
    printf("car motor & servo demo start\r\n");

    while (1) {
        /* 车轮前进 3 秒（osDelay 1 个单位 = 10ms，300 = 3 秒） */
        car_forward();
        printf("car forward\r\n");
        osDelay(300);

        /* 车顶舵机左转 */
        engine_turn_left();
        printf("servo turn left\r\n");
        osDelay(100);

        /* 车顶舵机回中 */
        regress_middle();
        printf("servo back to middle\r\n");
        osDelay(100);

        /* 车顶舵机右转 */
        engine_turn_right();
        printf("servo turn right\r\n");
        osDelay(100);

        /* 车轮停止 1 秒 */
        car_stop();
        printf("car stop\r\n");
        osDelay(100);
    }
}

APP_FEATURE_INIT(CarMain);
