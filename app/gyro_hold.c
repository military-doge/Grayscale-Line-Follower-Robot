#include "gyro_hold.h"
#include "jy62.h"

#define K_YAW        0.005f
#define YAW_DEADZONE 2.0f

static float   g_yaw_reference = 0.0f;
static uint8_t g_hold_active   = 0;

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

    /* Handle angle wrap-around at ±180° */
    if (error > 180.0f)
        error -= 360.0f;
    else if (error < -180.0f)
        error += 360.0f;

    /* Deadzone: tiny heading drift → no correction */
    if (error < YAW_DEADZONE && error > -YAW_DEADZONE)
        return 0.0f;

    return error * K_YAW;
}
