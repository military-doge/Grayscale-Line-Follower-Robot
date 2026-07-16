#include "task_planner.h"
#include "control.h"
#include "gyro_hold.h"
#include "jy62.h"
#include "grayscale.h"

/* Tunable constants */
#define TASK_STRAIGHT_SPEED   0.20f   /* Straight-line speed (m/s) */
#define TASK_DEBOUNCE_COUNT   3       /* Consecutive samples to confirm */

static TaskState g_task_state   = TASK_IDLE;
static uint8_t   g_black_count  = 0;
static uint8_t   g_key_pending  = 0;

/* ------------------------------------------------------------------ */

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

/* ------------------------------------------------------------------ */

void Task_Planner_Init(void)
{
    g_task_state  = TASK_IDLE;
    g_black_count = 0;
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
    float correction;

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
            Gyro_Hold_Set_Reference();
            g_black_count = 0;
            g_task_state  = TASK_PHASE1_STRAIGHT;
        }
        break;
    }
}
