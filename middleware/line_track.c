#include "line_track.h"
#include "control.h"
#include "jy62.h"

int g_line_error = 0;

/* 8路传感器权重表 */
static const int8_t g_track_weights[8] = {
    TRACK_W0, TRACK_W1, TRACK_W2, TRACK_W3,
    TRACK_W4, TRACK_W5, TRACK_W6, TRACK_W7
};

/* Control state */
static int      g_prev_error  = 0;
static uint8_t  g_curve_count = 0;

void Line_Tracking_Init(void)
{
    g_line_error  = 0;
    g_prev_error  = 0;
    g_curve_count = 0;
}

void Tracking_Reset_PID(void)
{
    g_line_error = 0;
    g_prev_error = 0;
}

int Tracking_Get_Last_Error(void)
{
    return g_line_error;
}

/**************************************************************************
 * 循迹更新函数 (在10ms定时器中断中调用)
 *
 * 纯PD控制 + 陀螺仪角速度阻尼
 * 无航向锁定 (航向锁定由 task_planner 在直行段管理, 循迹时禁用避免打架)
 *
 * 特殊情形:
 *   全部检测到(十字路口) -> 直行
 *   全部未检测到(丢线)    -> 直行, 依靠惯性找回
 **************************************************************************/
void Line_Tracking_Update(uint16_t *sensor_data)
{
    int weighted_sum = 0;
    int sum = 0;
    int error, abs_error;
    float correction, derivative, base_speed;
    uint8_t is_curve;

    /* Step 1: 加权平均法计算偏差 */
    for (uint8_t i = 0; i < 8; i++)
    {
        if (sensor_data[i])
        {
            weighted_sum += g_track_weights[i];
            sum++;
        }
    }

    if (sum == 0 || sum == 8)
    {
        /* 丢线或十字路口 → 直行 */
        g_line_error = 0;
        MotorA.Target_Encoder = TRACK_BASE_SPEED;
        MotorB.Target_Encoder = TRACK_BASE_SPEED;
        return;
    }

    g_line_error = weighted_sum / sum; /* 范围: -7 ~ +7 */

    error     = g_line_error;
    abs_error = (error > 0) ? error : -error;

    /* 直道死区: 小误差直行, 不修正 */
    if (abs_error <= TRACK_DEADZONE)
    {
        g_prev_error = 0;
        MotorA.Target_Encoder = TRACK_BASE_SPEED;
        MotorB.Target_Encoder = TRACK_BASE_SPEED;
        return;
    }

    /* === 弯道检测(带防抖) === */
    if (sensor_data[0] == ACTIVE_LEVEL || sensor_data[7] == ACTIVE_LEVEL
        || abs_error >= TRACK_CURVE_THRESHOLD)
    {
        if (g_curve_count < 255) g_curve_count++;
    }
    else
    {
        g_curve_count = 0;
    }
    is_curve = (g_curve_count >= 2);

    /* === 误差微分 === */
    derivative = (float)(error - g_prev_error);
    g_prev_error = error;

    /* === 非线性P + D === */
    if (is_curve)
    {
        correction = TRACK_KP * (float)error * (float)abs_error / 7.0f
                   + TRACK_KD * derivative;
        base_speed = TRACK_CURVE_SPEED;
    }
    else
    {
        correction = TRACK_KP * (float)error * (float)abs_error / 7.0f
                   + TRACK_KD * derivative;
        base_speed = TRACK_BASE_SPEED;
    }

    /* === 陀螺仪角速度阻尼 (仅抑制急转, 不锁航向) === */
    correction -= TRACK_KGYRO * JY62_Get_AngularVelocityZ();

    /* === 差速转向 === */
    MotorA.Target_Encoder = base_speed + correction;
    MotorB.Target_Encoder = base_speed - correction;

    /* === 速度限幅 (不倒转, 不超速) === */
    if (MotorA.Target_Encoder > TRACK_SPEED_MAX)  MotorA.Target_Encoder = TRACK_SPEED_MAX;
    if (MotorA.Target_Encoder < 0.0f)             MotorA.Target_Encoder = 0.0f;
    if (MotorB.Target_Encoder > TRACK_SPEED_MAX)  MotorB.Target_Encoder = TRACK_SPEED_MAX;
    if (MotorB.Target_Encoder < 0.0f)             MotorB.Target_Encoder = 0.0f;
}
