#ifndef __LINE_PATROL_H__
#define __LINE_PATROL_H__

/*
 * 黑胶带循迹巡航线程（Y 路口随机选路 / 干字型终点停车 / 死路掉头）
 * 实现见 line_patrol.c，可调参数（速度、路口判定时长、死路超时等）都在 .c 顶部「实车标定区」。
 */
void LinePatrol_Thread(void *arg);

#endif
