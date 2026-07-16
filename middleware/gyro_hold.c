#include "gyro_hold.h"
#include "jy62.h"

#define K_YAW         0.005f
#define YAW_DEADZONE  2.0f

static float   g_yaw_reference = 0.0f;
static uint8_t g_hold_active   = 0;

/* ------------------------------------------------------------------ */

static float normalize_180(float angle)
{
    if (angle > 180.0f)  return angle - 360.0f;
    if (angle < -180.0f) return angle + 360.0f;
    return angle;
}

/* ------------------------------------------------------------------ */

void Gyro_Hold_Init(void)
{
    g_yaw_reference = 0.0f;
    g_hold_active   = 0;
}

void Gyro_Hold_Set_Reference(void)
{
    g_yaw_reference = JY62_Get_Yaw();
    g_hold_active   = 1;
}

void Gyro_Hold_Clear(void)
{
    g_hold_active = 0;
}

float Gyro_Hold_Get_Correction(void)
{
    float error;

    if (!g_hold_active || !JY62_Is_Data_Ready())
        return 0.0f;

    error = JY62_Get_Yaw() - g_yaw_reference;
    error = normalize_180(error);

    if (error < YAW_DEADZONE && error > -YAW_DEADZONE)
        return 0.0f;

    return error * K_YAW;
}

float Gyro_Hold_Get_Error(void)
{
    if (!JY62_Is_Data_Ready())
        return 0.0f;

    return normalize_180(JY62_Get_Yaw() - g_yaw_reference);
}
