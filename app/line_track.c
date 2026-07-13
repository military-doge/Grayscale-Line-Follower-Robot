#include "line_track.h"
#include "control.h"

int g_line_error = 0;

/* 8路传感器权重表 */
static const int8_t g_track_weights[8] = {
    TRACK_W0, TRACK_W1, TRACK_W2, TRACK_W3,
    TRACK_W4, TRACK_W5, TRACK_W6, TRACK_W7
};

/* 控制状态变量 */
static int g_prev_error = 0;

void Line_Tracking_Init(void)
{
    g_line_error = 0;
    g_prev_error = 0;
}

/**************************************************************************
 * 复位PID状态 (启停时调用)
 **************************************************************************/
void Tracking_Reset_PID(void)
{
    g_line_error = 0;
    g_prev_error = 0;
}

/**************************************************************************
 * 获取上一次计算的误差值 (供OLED显示/调试用)
 **************************************************************************/
int Tracking_Get_Last_Error(void)
{
    return g_line_error;
}

/**************************************************************************
 * 循迹更新函数 (在10ms定时器中断中调用)
 *
 * 非线性PD控制:
 *   correction = KP * error * |error|/7 + KD * derivative
 *   统一处理直道和弯道, 无模式切换, 无死区
 *
 * 特殊情形:
 *   全部检测到(十字路口) -> 直行
 *   全部未检测到(丢线)    -> 保持上次误差
 **************************************************************************/
void Line_Tracking_Update(uint16_t *sensor_data)
{
    int weighted_sum = 0;
    int sum = 0;
    int error, abs_error;
    float correction, derivative;
    uint8_t i;

    /* Step 1: 加权平均法计算偏差 */
    for (i = 0; i < 8; i++)
    {
        if (sensor_data[i])
        {
            weighted_sum += g_track_weights[i];
            sum++;
        }
    }

    /* 全部未检测到黑线: 保持上一次的误差方向 */
    if (sum == 0)
    {
        /* g_line_error 保持不变 */
    }
    /* 全部检测到黑线 (十字/粗线): 直行 */
    else if (sum == 8)
    {
        g_line_error = 0;
    }
    else
    {
        g_line_error = weighted_sum / sum; /* 范围: -7 ~ +7 */
    }

    error = g_line_error;
    abs_error = (error > 0) ? error : -error;

    /* Step 2: 微分计算 */
    derivative = (float)(error - g_prev_error);
    g_prev_error = error;

    /* Step 3: 非线性PD */
    correction = TRACK_KP * (float)error * (float)abs_error / 7.0f
               + TRACK_KD * derivative;

    /* Step 4: 差速转向 */
    MotorA.Target_Encoder = TRACK_BASE_SPEED + correction;
    MotorB.Target_Encoder = TRACK_BASE_SPEED - correction;

    /* Step 5: 限幅 */
    if (MotorA.Target_Encoder > TRACK_SPEED_MAX)  MotorA.Target_Encoder = TRACK_SPEED_MAX;
    if (MotorA.Target_Encoder < -TRACK_SPEED_MAX) MotorA.Target_Encoder = -TRACK_SPEED_MAX;
    if (MotorB.Target_Encoder > TRACK_SPEED_MAX)  MotorB.Target_Encoder = TRACK_SPEED_MAX;
    if (MotorB.Target_Encoder < -TRACK_SPEED_MAX) MotorB.Target_Encoder = -TRACK_SPEED_MAX;
}
