#include "task_planner.h"
#include "control.h"
#include "gyro_hold.h"
#include "jy62.h"
#include "grayscale.h"
#include "line_track.h"

/* Tunable constants */
#define TASK_STRAIGHT_SPEED   0.20f   /* Straight-line speed (m/s) */
#define TASK_ALIGN_SPEED      0.02f   /* In-place rotation speed (m/s) */
#define TASK_ALIGN_THRESHOLD  5.0f    /* Heading-alignment tolerance (°) */
#define TASK_DEBOUNCE_COUNT   8       /* Consecutive samples to confirm (80ms) */

static TaskState g_task_state   = TASK_IDLE;
static float     g_initial_yaw  = 0.0f;
static float     g_target_yaw   = 0.0f;
static uint8_t   g_black_count  = 0;
static uint8_t   g_white_count  = 0;
static uint8_t   g_key_pending  = 0;

/* ------------------------------------------------------------------ */

static float normalize_180(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static uint8_t count_black(uint16_t *sensor_data)
{
    uint8_t i, cnt = 0;
    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (sensor_data[i]) cnt++;
    }
    return cnt;
}

/* At least 2 channels must see black (filters single-sensor noise) */
static uint8_t is_any_black(uint16_t *sensor_data)
{
    return count_black(sensor_data) >= 2;
}

/* All channels white = line truly gone (no sensor sees black) */
static uint8_t is_all_white(uint16_t *sensor_data)
{
    return count_black(sensor_data) == 0;
}

/* ------------------------------------------------------------------ */

void Task_Planner_Init(void)
{
    g_task_state  = TASK_IDLE;
    g_initial_yaw = 0.0f;
    g_target_yaw  = 0.0f;
    g_black_count = 0;
    g_white_count = 0;
    g_key_pending = 0;
}

void Task_Planner_On_Key(void)
{
    g_key_pending = 1;
}

TaskState Task_Planner_Get_State(void)
{
    return g_task_state;
}

/* ------------------------------------------------------------------ */

void Task_Planner_Update(uint16_t *sensor_data)
{
    float correction, yaw_error;

    switch (g_task_state) {

    /* ============================================================== */
    case TASK_IDLE:
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        if (g_key_pending) {
            g_key_pending = 0;
            Gyro_Hold_Set_Reference();
            g_black_count = 0;
            g_task_state  = TASK_PHASE1_STRAIGHT;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE1_STRAIGHT:
        correction = Gyro_Hold_Get_Correction();
        MotorA.Target_Encoder = TASK_STRAIGHT_SPEED + correction;
        MotorB.Target_Encoder = TASK_STRAIGHT_SPEED - correction;

        if (is_any_black(sensor_data)) {
            g_black_count++;
            if (g_black_count >= TASK_DEBOUNCE_COUNT) {
                MotorA.Target_Encoder = 0.0f;
                MotorB.Target_Encoder = 0.0f;
                Gyro_Hold_Clear();
                g_key_pending = 0;   /* discard stale key events */
                g_task_state = TASK_PHASE1_STOPPED;
            }
        } else {
            g_black_count = 0;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE1_STOPPED:
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        if (g_key_pending) {
            g_key_pending = 0;
            /* Record initial heading for later 180° turn */
            g_initial_yaw = JY62_Get_Yaw();
            Gyro_Hold_Set_Reference();
            g_black_count = 0;
            g_white_count = 0;
            g_task_state  = TASK_PHASE2_STRAIGHT1;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE2_STRAIGHT1:
        correction = Gyro_Hold_Get_Correction();
        MotorA.Target_Encoder = TASK_STRAIGHT_SPEED + correction;
        MotorB.Target_Encoder = TASK_STRAIGHT_SPEED - correction;

        if (is_any_black(sensor_data)) {
            g_black_count++;
            if (g_black_count >= TASK_DEBOUNCE_COUNT) {
                Gyro_Hold_Clear();   /* release heading hold, let tracking steer */
                Tracking_Reset_PID();
                g_white_count = 0;
                g_task_state  = TASK_PHASE2_TRACK1;
            }
        } else {
            g_black_count = 0;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE2_TRACK1:
        Line_Tracking_Update(sensor_data);

        if (is_all_white(sensor_data)) {
            g_white_count++;
            if (g_white_count >= TASK_DEBOUNCE_COUNT) {
                /* Turn 180° from initial direction */
                g_target_yaw = normalize_180(g_initial_yaw + 180.0f);
                g_task_state = TASK_PHASE2_ALIGN;
            }
        } else {
            g_white_count = 0;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE2_ALIGN:
        if (JY62_Is_Data_Ready()) {
            yaw_error = normalize_180(g_target_yaw - JY62_Get_Yaw());

            if (yaw_error > -TASK_ALIGN_THRESHOLD && yaw_error < TASK_ALIGN_THRESHOLD) {
                /* Heading aligned — lock and go straight back */
                Gyro_Hold_Set_Reference();
                g_black_count = 0;
                g_task_state  = TASK_PHASE2_STRAIGHT2;
            } else if (yaw_error > 0) {
                /* Rotate CCW (+yaw): left backward, right forward */
                MotorA.Target_Encoder = -TASK_ALIGN_SPEED;
                MotorB.Target_Encoder =  TASK_ALIGN_SPEED;
            } else {
                /* Rotate CW (-yaw): left forward, right backward */
                MotorA.Target_Encoder =  TASK_ALIGN_SPEED;
                MotorB.Target_Encoder = -TASK_ALIGN_SPEED;
            }
        } else {
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = 0.0f;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE2_STRAIGHT2:
        correction = Gyro_Hold_Get_Correction();
        MotorA.Target_Encoder = TASK_STRAIGHT_SPEED + correction;
        MotorB.Target_Encoder = TASK_STRAIGHT_SPEED - correction;

        if (is_any_black(sensor_data)) {
            g_black_count++;
            if (g_black_count >= TASK_DEBOUNCE_COUNT) {
                Gyro_Hold_Clear();   /* release heading hold, let tracking steer */
                Tracking_Reset_PID();
                g_white_count = 0;
                g_task_state  = TASK_PHASE2_TRACK2;
            }
        } else {
            g_black_count = 0;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE2_TRACK2:
        Line_Tracking_Update(sensor_data);

        if (is_all_white(sensor_data)) {
            g_white_count++;
            if (g_white_count >= TASK_DEBOUNCE_COUNT) {
                MotorA.Target_Encoder = 0.0f;
                MotorB.Target_Encoder = 0.0f;
                g_key_pending = 0;   /* discard stale key events */
                g_task_state = TASK_PHASE2_DONE;
            }
        } else {
            g_white_count = 0;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE2_DONE:
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        break;
    }
}
