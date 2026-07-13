#ifndef _LINE_TRACK_H
#define _LINE_TRACK_H

#include <stdint.h>

/* 8-channel symmetric sensor weights (ch0~ch7 left to right) */
#define TRACK_W0  (-7)
#define TRACK_W1  (-5)
#define TRACK_W2  (-3)
#define TRACK_W3  (-1)
#define TRACK_W4  (1)
#define TRACK_W5  (3)
#define TRACK_W6  (5)
#define TRACK_W7  (7)

/*
 * 循迹控制参数 (非线性PD控制)
 * 误差范围: -7 ~ +7 (加权平均)
 *
 * 非线性比例: 修正 = KP * error * |error| / 7
 *   小误差温和修正, 大误差强修正, 兼顾直道稳定和弯道转向
 */

/* 比例系数 (非线性, 实际增益 = KP * |error| / 7) */
#define TRACK_KP             0.12f
/* 微分系数 */
#define TRACK_KD             0.08f
/* 积分系数 (未使用) */
#define TRACK_KI             0.0f

#define TRACK_BASE_SPEED     0.08f    /* 循迹速度 (m/s) */
#define TRACK_SPEED_MAX      0.35f    /* 最高速度限幅 (m/s) */

#define TRACK_INTEGRAL_LIMIT 200.0f   /* 积分限幅 */

extern int g_line_error;

void Line_Tracking_Init(void);
void Line_Tracking_Update(uint16_t *sensor_data);
void Tracking_Reset_PID(void);
int Tracking_Get_Last_Error(void);

#endif /* _LINE_TRACK_H */
