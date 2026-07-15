#include "line_track.h"
#include "control.h"
#include "gyro_hold.h"

int g_line_error = 0;

/* 8路传感器权重表 */
static const int8_t g_track_weights[8] = {
    TRACK_W0, TRACK_W1, TRACK_W2, TRACK_W3,
    TRACK_W4, TRACK_W5, TRACK_W6, TRACK_W7
};

/* Control state */
static int   g_prev_error = 0;
static uint8_t g_curve_count  = 0;   /* Consecutive curve detection counter */
static uint8_t g_was_straight = 0;   /* Previous straight-line state */

void Line_Tracking_Init(void)
{
    g_line_error   = 0;
    g_prev_error   = 0;
    g_curve_count  = 0;
    g_was_straight = 0;
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
    float correction, derivative, base_speed;
    uint8_t i, is_curve;

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

    if (sum == 0 || sum == 8)
    {
        /* === Line lost / crossroads → pure gyro heading hold === */
        float gyro_correction = Gyro_Hold_Get_Correction();
        g_was_straight = 0;
        MotorA.Target_Encoder = TRACK_BASE_SPEED + gyro_correction;
        MotorB.Target_Encoder = TRACK_BASE_SPEED - gyro_correction;
    }
    else
    {
        /* === Straight-line entry detection → record yaw reference === */
        {
            uint8_t is_straight = (abs_error <= TRACK_DEADZONE);
            if (is_straight && !g_was_straight)
                Gyro_Hold_Set_Reference();
            g_was_straight = is_straight;
        }

        /* === Curve detection with debounce === */
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

        /* === Derivative of error === */
        derivative = (float)(error - g_prev_error);
        g_prev_error = error;

        /* === Nonlinear P + D, dual-mode control === */
        if (is_curve)
        {
            correction = TRACK_KP * (float)error * (float)abs_error / 7.0f
                       + TRACK_KD * derivative;
            base_speed = TRACK_CURVE_SPEED;
        }
        else
        {
            if (abs_error <= TRACK_DEADZONE)
            {
                MotorA.Target_Encoder = TRACK_BASE_SPEED;
                MotorB.Target_Encoder = TRACK_BASE_SPEED;
                return;
            }
            correction = TRACK_KP * (float)error * (float)abs_error / 7.0f
                       + TRACK_KD * derivative;
            base_speed = TRACK_BASE_SPEED;
        }

        /* === Gyro-assisted correction === */
        correction -= TRACK_KGYRO * JY62_Get_AngularVelocityZ();
        correction += Gyro_Hold_Get_Correction();

        /* === Differential steering === */
        MotorA.Target_Encoder = base_speed + correction;
        MotorB.Target_Encoder = base_speed - correction;
    }

    /* === Speed clamp === */
    if (MotorA.Target_Encoder > TRACK_SPEED_MAX)  MotorA.Target_Encoder = TRACK_SPEED_MAX;
    if (MotorA.Target_Encoder < -TRACK_SPEED_MAX) MotorA.Target_Encoder = -TRACK_SPEED_MAX;
    if (MotorB.Target_Encoder > TRACK_SPEED_MAX)  MotorB.Target_Encoder = TRACK_SPEED_MAX;
    if (MotorB.Target_Encoder < -TRACK_SPEED_MAX) MotorB.Target_Encoder = -TRACK_SPEED_MAX;
}
