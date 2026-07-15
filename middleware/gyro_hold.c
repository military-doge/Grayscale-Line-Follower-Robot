#include "gyro_hold.h"
#include "jy62.h"
#include "board.h"

#define K_YAW             0.005f
#define YAW_DEADZONE      2.0f
#define STRAIGHT_SPEED    0.10f
#define ROT_SPEED         0.03f
#define ROT_DEADZONE      3.0f
#define ARCTAN_08         38.66f

static float   g_yaw_reference = 0.0f;
static uint8_t g_hold_active   = 0;

/* Sequence state machine */
typedef enum {
    GS_IDLE = 0,

    /* Mode 0: straight → track → stop */
    GS_SIMPLE,
    GS_SIMPLE_TRACK,

    /* Mode 1: straight → track → straight → track → stop */
    GS_C1_STRAIGHT, GS_C1_TRACK,
    GS_C2_STRAIGHT, GS_C2_TRACK,

    /* Mode 2: pattern ×4 */
    GS_PAT_INIT,
    GS_PAT_ROT_R,
    GS_PAT_S1,
    GS_PAT_T1,
    GS_PAT_ADJ_OPP,
    GS_PAT_ROT_L,
    GS_PAT_S2,
    GS_PAT_T2,
    GS_PAT_ADJ_INIT,
    GS_PAT_CHECK,

    GS_DONE
} GyroStraightState;

static GyroStraightState g_state = GS_IDLE;
static uint8_t g_press_mode = 0;

/* Mode 2 persistent variables */
static float   g_initial_yaw  = 0.0f;
static float   g_target_yaw   = 0.0f;
static uint8_t g_repeat_count = 0;

/* ------------------------------------------------------------------ */

static float normalize_180(float angle)
{
    if (angle > 180.0f)  return angle - 360.0f;
    if (angle < -180.0f) return angle + 360.0f;
    return angle;
}

/* In-place rotation to g_target_yaw. Returns 1 when within deadzone. */
static uint8_t rotate_to_target(void)
{
    float error = g_target_yaw - JY62_Get_Yaw();
    float speed;

    error = normalize_180(error);

    if (error < ROT_DEADZONE && error > -ROT_DEADZONE) {
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        return 1;
    }

    speed = (error > 0) ? ROT_SPEED : -ROT_SPEED;
    MotorA.Target_Encoder =  speed;
    MotorB.Target_Encoder = -speed;
    return 0;
}

/* ------------------------------------------------------------------ */

void Gyro_Hold_Init(void)
{
    g_yaw_reference = 0.0f;
    g_hold_active   = 0;
    g_state         = GS_IDLE;
    g_press_mode    = 0;
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

/* ------------------------------------------------------------------ */

void Gyro_Straight_Start(void)
{
    g_yaw_reference = JY62_Get_Yaw();
    g_hold_active   = 1;

    switch (g_press_mode) {
    case 0: g_state = GS_SIMPLE;      break;
    case 1: g_state = GS_C1_STRAIGHT; break;
    case 2: g_state = GS_PAT_INIT;    break;
    }

    g_press_mode = (g_press_mode + 1) % 3;
}

/* ------------------------------------------------------------------ */

uint8_t Gyro_Straight_Update(uint16_t *sensor_data)
{
    uint8_t i, sum = 0;
    float gyro_correction;

    if (g_state == GS_IDLE || g_state == GS_DONE)
        return (g_state == GS_DONE);

    /* Count active (black) sensors */
    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (sensor_data[i] == ACTIVE_LEVEL) sum++;
    }

    switch (g_state) {

    /* ======= Mode 0: straight until black → track → done ======= */
    case GS_SIMPLE:
        if (sum > 0) {
            Tracking_Reset_PID();
            g_state = GS_SIMPLE_TRACK;
            /* fall through to start tracking immediately */
        } else {
            gyro_correction = Gyro_Hold_Get_Correction();
            MotorA.Target_Encoder = STRAIGHT_SPEED + gyro_correction;
            MotorB.Target_Encoder = STRAIGHT_SPEED - gyro_correction;
            break;
        }
        /* fall through */

    case GS_SIMPLE_TRACK:
        Line_Tracking_Update(sensor_data);
        if (sum == 0) {
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = 0.0f;
            g_hold_active = 0;
            g_state = GS_DONE;
            return 1;
        }
        break;

    /* ======= Mode 1: straight → track → straight → track ======= */
    case GS_C1_STRAIGHT:
        if (sum > 0) {
            Tracking_Reset_PID();
            g_state = GS_C1_TRACK;
        } else {
            gyro_correction = Gyro_Hold_Get_Correction();
            MotorA.Target_Encoder = STRAIGHT_SPEED + gyro_correction;
            MotorB.Target_Encoder = STRAIGHT_SPEED - gyro_correction;
            break;
        }
        /* fall through */

    case GS_C1_TRACK:
        if (sum == 0) {
            g_yaw_reference = JY62_Get_Yaw();
            g_hold_active   = 1;
            g_state = GS_C2_STRAIGHT;
            gyro_correction = Gyro_Hold_Get_Correction();
            MotorA.Target_Encoder = STRAIGHT_SPEED + gyro_correction;
            MotorB.Target_Encoder = STRAIGHT_SPEED - gyro_correction;
            break;
        }
        Line_Tracking_Update(sensor_data);
        break;

    case GS_C2_STRAIGHT:
        if (sum > 0) {
            Tracking_Reset_PID();
            g_state = GS_C2_TRACK;
        } else {
            gyro_correction = Gyro_Hold_Get_Correction();
            MotorA.Target_Encoder = STRAIGHT_SPEED + gyro_correction;
            MotorB.Target_Encoder = STRAIGHT_SPEED - gyro_correction;
            break;
        }
        /* fall through */

    case GS_C2_TRACK:
        if (sum == 0) {
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = 0.0f;
            g_hold_active = 0;
            g_state = GS_DONE;
            return 1;
        }
        Line_Tracking_Update(sensor_data);
        break;

    /* ======= Mode 2: pattern ×4 ======= */
    case GS_PAT_INIT:
        g_initial_yaw  = JY62_Get_Yaw();
        g_repeat_count = 4;
        g_target_yaw   = normalize_180(g_initial_yaw - ARCTAN_08);
        g_state = GS_PAT_ROT_R;
        /* fall through */

    case GS_PAT_ROT_R:
        if (rotate_to_target()) {
            g_yaw_reference = JY62_Get_Yaw();
            g_hold_active   = 1;
            g_state = GS_PAT_S1;
        }
        break;

    case GS_PAT_S1:
        if (sum > 0) {
            Tracking_Reset_PID();
            g_state = GS_PAT_T1;
        } else {
            gyro_correction = Gyro_Hold_Get_Correction();
            MotorA.Target_Encoder = STRAIGHT_SPEED + gyro_correction;
            MotorB.Target_Encoder = STRAIGHT_SPEED - gyro_correction;
            break;
        }
        /* fall through */

    case GS_PAT_T1:
        if (sum == 0) {
            g_target_yaw = normalize_180(g_initial_yaw + 180.0f);
            g_state = GS_PAT_ADJ_OPP;
            break;
        }
        Line_Tracking_Update(sensor_data);
        break;

    case GS_PAT_ADJ_OPP:
        if (rotate_to_target()) {
            g_target_yaw = normalize_180(g_initial_yaw + 180.0f - ARCTAN_08);
            g_state = GS_PAT_ROT_L;
        }
        break;

    case GS_PAT_ROT_L:
        if (rotate_to_target()) {
            g_yaw_reference = JY62_Get_Yaw();
            g_hold_active   = 1;
            g_state = GS_PAT_S2;
        }
        break;

    case GS_PAT_S2:
        if (sum > 0) {
            Tracking_Reset_PID();
            g_state = GS_PAT_T2;
        } else {
            gyro_correction = Gyro_Hold_Get_Correction();
            MotorA.Target_Encoder = STRAIGHT_SPEED + gyro_correction;
            MotorB.Target_Encoder = STRAIGHT_SPEED - gyro_correction;
            break;
        }
        /* fall through */

    case GS_PAT_T2:
        if (sum == 0) {
            g_target_yaw = g_initial_yaw;
            g_state = GS_PAT_ADJ_INIT;
            break;
        }
        Line_Tracking_Update(sensor_data);
        break;

    case GS_PAT_ADJ_INIT:
        if (rotate_to_target()) {
            g_state = GS_PAT_CHECK;
        }
        break;

    case GS_PAT_CHECK:
        if (--g_repeat_count > 0) {
            g_target_yaw = normalize_180(g_initial_yaw - ARCTAN_08);
            g_state = GS_PAT_ROT_R;
        } else {
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = 0.0f;
            g_hold_active = 0;
            g_state = GS_DONE;
            return 1;
        }
        break;

    default:
        break;
    }

    /* Speed clamp */
    if (MotorA.Target_Encoder > TRACK_SPEED_MAX)  MotorA.Target_Encoder = TRACK_SPEED_MAX;
    if (MotorA.Target_Encoder < -TRACK_SPEED_MAX) MotorA.Target_Encoder = -TRACK_SPEED_MAX;
    if (MotorB.Target_Encoder > TRACK_SPEED_MAX)  MotorB.Target_Encoder = TRACK_SPEED_MAX;
    if (MotorB.Target_Encoder < -TRACK_SPEED_MAX) MotorB.Target_Encoder = -TRACK_SPEED_MAX;

    return 0;
}

uint8_t Gyro_Straight_Is_Active(void)
{
    return (g_state != GS_IDLE && g_state != GS_DONE);
}
