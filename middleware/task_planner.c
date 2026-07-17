#include "task_planner.h"
#include "control.h"
#include "gyro_hold.h"
#include "jy62.h"
#include "grayscale.h"
#include "line_track.h"

/* Tunable constants */
#define TASK_STRAIGHT_SPEED   0.30f   /* Straight-line speed (m/s) */
#define TASK_ALIGN_SPEED      0.02f   /* In-place rotation speed (m/s) */
#define TASK_ALIGN_THRESHOLD  5.0f    /* Heading-alignment tolerance (°) */
#define TASK_DEBOUNCE_COUNT   4       /* Consecutive samples to confirm (40ms) */
#define ARCTAN_0_8_DEG               38.66f   /* Turn angle in degrees */
#define TASK_PAUSE_COUNT             50       /* 50 * 10ms = 500ms pause after turn */
#define TASK_DISTANCE_PULSES_110CM   7585     /* 4137*110/120 * 2 wheels */
#define TASK_SLOW_SPEED              0.08f    /* Reduced speed after 110cm (m/s) */
#define TASK_TRACK_ENTRY_SPEED        0.05f    /* Slow speed when entering track */
#define TASK_CRAWL_PULSES_4CM        276      /* 40mm / 0.29mm * 2 wheels */

/* Straight-drive sub-states (Phase 3) */
#define STRAIGHT_SUB_DRIVE            0
#define STRAIGHT_SUB_SLOW             1

static TaskState g_task_state   = TASK_IDLE;
static float     g_initial_yaw  = 0.0f;
static float     g_target_yaw   = 0.0f;
static uint8_t   g_black_count  = 0;
static uint8_t   g_white_count  = 0;
static uint8_t   g_key_pending  = 0;
static uint8_t   g_phase3_loop     = 0;
static uint8_t   g_pause_count     = 0;
static int32_t   g_distance_pulses = 0;
static uint8_t   g_straight_sub    = STRAIGHT_SUB_DRIVE;
static uint8_t   g_slow_timer      = 0;
static int32_t   g_crawl_pulses    = 0;

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

/* At least 1 channel must see black */
static uint8_t is_any_black(uint16_t *sensor_data)
{
    return count_black(sensor_data) >= 1;
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
    g_key_pending  = 0;
    g_phase3_loop  = 0;
    g_pause_count  = 0;
    g_distance_pulses = 0;
    g_straight_sub    = STRAIGHT_SUB_DRIVE;
    g_slow_timer      = 0;
    g_crawl_pulses    = 0;
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

void Task_Planner_Update(uint16_t *sensor_data, int encoder_a, int encoder_b)
{
    float correction, yaw_error;

    switch (g_task_state) {

    /* ============================================================== */
    case TASK_IDLE:
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        if (g_key_pending) {
            g_key_pending = 0;
            g_initial_yaw = JY62_Get_Yaw();
            g_phase3_loop = 0;
            g_black_count = 0;
            g_target_yaw  = normalize_180(JY62_Get_Yaw() - ARCTAN_0_8_DEG);
            g_task_state  = TASK_PHASE3_RTURN_ALIGN;
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
                g_key_pending = 0;
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
        if (g_key_pending) {
            g_key_pending = 0;
            g_phase3_loop = 0;
            g_black_count = 0;
            g_target_yaw  = normalize_180(JY62_Get_Yaw() - ARCTAN_0_8_DEG);
            g_task_state  = TASK_PHASE3_RTURN_ALIGN;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE3_RTURN_ALIGN:
        if (JY62_Is_Data_Ready()) {
            yaw_error = normalize_180(g_target_yaw - JY62_Get_Yaw());

            if (yaw_error > -TASK_ALIGN_THRESHOLD && yaw_error < TASK_ALIGN_THRESHOLD) {
                MotorA.Target_Encoder = 0.0f;
                MotorB.Target_Encoder = 0.0f;
                if (g_pause_count == 0) {
                    Gyro_Hold_Set_Reference();
                }
                if (++g_pause_count >= TASK_PAUSE_COUNT) {
                    g_black_count      = 0;
                    g_pause_count      = 0;
                    g_distance_pulses  = 0;
                    g_straight_sub     = STRAIGHT_SUB_DRIVE;
                    g_task_state       = TASK_PHASE3_STRAIGHT1;
                }
            } else if (yaw_error > 0) {
                g_pause_count = 0;
                MotorA.Target_Encoder = -TASK_ALIGN_SPEED;
                MotorB.Target_Encoder =  TASK_ALIGN_SPEED;
            } else {
                g_pause_count = 0;
                MotorA.Target_Encoder =  TASK_ALIGN_SPEED;
                MotorB.Target_Encoder = -TASK_ALIGN_SPEED;
            }
        } else {
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = 0.0f;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE3_STRAIGHT1:
        switch (g_straight_sub) {

        case STRAIGHT_SUB_DRIVE:
            g_distance_pulses += (encoder_a > 0 ? encoder_a : -encoder_a)
                               + (encoder_b > 0 ? encoder_b : -encoder_b);

            if (is_any_black(sensor_data)) {
                /* Immediately release gyro, slow down and steer toward line */
                Gyro_Hold_Clear();
                {
                    int wsum = 0, scnt = 0;
                    static const int8_t w[8] = {-7,-5,-3,-1,1,3,5,7};
                    for (uint8_t i = 0; i < 8; i++) {
                        if (sensor_data[i]) { wsum += w[i]; scnt++; }
                    }
                    float steer = 0.008f * (float)wsum / (float)scnt - 0.012f;
                    MotorA.Target_Encoder = TASK_TRACK_ENTRY_SPEED + steer;
                    MotorB.Target_Encoder = TASK_TRACK_ENTRY_SPEED - steer;
                }
                g_black_count++;
                if (g_black_count >= TASK_DEBOUNCE_COUNT) {
                    g_black_count  = 0;
                    g_straight_sub = STRAIGHT_SUB_SLOW;
                    g_slow_timer   = 0;
                    Tracking_Reset_PID();
                }
            } else {
                g_black_count = 0;
                correction = Gyro_Hold_Get_Correction();
                if (g_distance_pulses >= TASK_DISTANCE_PULSES_110CM) {
                    MotorA.Target_Encoder = TASK_SLOW_SPEED + correction;
                    MotorB.Target_Encoder = TASK_SLOW_SPEED - correction;
                } else {
                    MotorA.Target_Encoder = TASK_STRAIGHT_SPEED + correction;
                    MotorB.Target_Encoder = TASK_STRAIGHT_SPEED - correction;
                }
            }
            break;

        case STRAIGHT_SUB_SLOW:
            Line_Tracking_Update(sensor_data);
            MotorA.Target_Encoder -= 0.01f;
            MotorB.Target_Encoder += 0.01f;
            {
                float m = MotorA.Target_Encoder > MotorB.Target_Encoder ? MotorA.Target_Encoder : MotorB.Target_Encoder;
                if (m > TASK_TRACK_ENTRY_SPEED) {
                    float s = TASK_TRACK_ENTRY_SPEED / m;
                    MotorA.Target_Encoder *= s;
                    MotorB.Target_Encoder *= s;
                }
            }
            if (MotorA.Target_Encoder < 0.0f) MotorA.Target_Encoder = 0.0f;
            if (MotorB.Target_Encoder < 0.0f) MotorB.Target_Encoder = 0.0f;
            if (++g_slow_timer >= 150) {
                g_white_count  = 0;
                g_straight_sub = STRAIGHT_SUB_DRIVE;
                g_task_state   = TASK_PHASE3_TRACK1;
            }
            break;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE3_TRACK1:
        Line_Tracking_Update(sensor_data);

        if (is_all_white(sensor_data)) {
            g_white_count++;
            if (g_white_count >= TASK_DEBOUNCE_COUNT) {
                g_target_yaw = normalize_180(g_initial_yaw + 180.0f);
                g_task_state = TASK_PHASE3_ALIGN180;
            }
        } else {
            g_white_count = 0;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE3_ALIGN180:
        if (JY62_Is_Data_Ready()) {
            yaw_error = normalize_180(g_target_yaw - JY62_Get_Yaw());

            if (yaw_error > -TASK_ALIGN_THRESHOLD && yaw_error < TASK_ALIGN_THRESHOLD) {
                g_target_yaw = normalize_180(g_initial_yaw + 180.0f + ARCTAN_0_8_DEG);
                g_task_state = TASK_PHASE3_LTURN_ALIGN;
            } else if (yaw_error > 0) {
                MotorA.Target_Encoder = -TASK_ALIGN_SPEED;
                MotorB.Target_Encoder =  TASK_ALIGN_SPEED;
            } else {
                MotorA.Target_Encoder =  TASK_ALIGN_SPEED;
                MotorB.Target_Encoder = -TASK_ALIGN_SPEED;
            }
        } else {
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = 0.0f;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE3_LTURN_ALIGN:
        if (JY62_Is_Data_Ready()) {
            yaw_error = normalize_180(g_target_yaw - JY62_Get_Yaw());

            if (yaw_error > -TASK_ALIGN_THRESHOLD && yaw_error < TASK_ALIGN_THRESHOLD) {
                MotorA.Target_Encoder = 0.0f;
                MotorB.Target_Encoder = 0.0f;
                if (g_pause_count == 0) {
                    Gyro_Hold_Set_Reference();
                }
                if (++g_pause_count >= TASK_PAUSE_COUNT) {
                    g_black_count      = 0;
                    g_pause_count      = 0;
                    g_distance_pulses  = 0;
                    g_straight_sub     = STRAIGHT_SUB_DRIVE;
                    g_task_state       = TASK_PHASE3_STRAIGHT2;
                }
            } else if (yaw_error > 0) {
                g_pause_count = 0;
                MotorA.Target_Encoder = -TASK_ALIGN_SPEED;
                MotorB.Target_Encoder =  TASK_ALIGN_SPEED;
            } else {
                g_pause_count = 0;
                MotorA.Target_Encoder =  TASK_ALIGN_SPEED;
                MotorB.Target_Encoder = -TASK_ALIGN_SPEED;
            }
        } else {
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = 0.0f;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE3_STRAIGHT2:
        switch (g_straight_sub) {

        case STRAIGHT_SUB_DRIVE:
            g_distance_pulses += (encoder_a > 0 ? encoder_a : -encoder_a)
                               + (encoder_b > 0 ? encoder_b : -encoder_b);

            if (is_any_black(sensor_data)) {
                /* Immediately release gyro, slow down and steer toward line */
                Gyro_Hold_Clear();
                {
                    int wsum = 0, scnt = 0;
                    static const int8_t w[8] = {-7,-5,-3,-1,1,3,5,7};
                    for (uint8_t i = 0; i < 8; i++) {
                        if (sensor_data[i]) { wsum += w[i]; scnt++; }
                    }
                    float steer = 0.008f * (float)wsum / (float)scnt + 0.012f;
                    MotorA.Target_Encoder = TASK_TRACK_ENTRY_SPEED + steer;
                    MotorB.Target_Encoder = TASK_TRACK_ENTRY_SPEED - steer;
                }
                g_black_count++;
                if (g_black_count >= TASK_DEBOUNCE_COUNT) {
                    g_black_count  = 0;
                    g_straight_sub = STRAIGHT_SUB_SLOW;
                    g_slow_timer   = 0;
                    Tracking_Reset_PID();
                }
            } else {
                g_black_count = 0;
                correction = Gyro_Hold_Get_Correction();
                if (g_distance_pulses >= TASK_DISTANCE_PULSES_110CM) {
                    MotorA.Target_Encoder = TASK_SLOW_SPEED + correction;
                    MotorB.Target_Encoder = TASK_SLOW_SPEED - correction;
                } else {
                    MotorA.Target_Encoder = TASK_STRAIGHT_SPEED + correction;
                    MotorB.Target_Encoder = TASK_STRAIGHT_SPEED - correction;
                }
            }
            break;

        case STRAIGHT_SUB_SLOW:
            Line_Tracking_Update(sensor_data);
            MotorA.Target_Encoder += 0.01f;
            MotorB.Target_Encoder -= 0.01f;
            {
                float m = MotorA.Target_Encoder > MotorB.Target_Encoder ? MotorA.Target_Encoder : MotorB.Target_Encoder;
                if (m > TASK_TRACK_ENTRY_SPEED) {
                    float s = TASK_TRACK_ENTRY_SPEED / m;
                    MotorA.Target_Encoder *= s;
                    MotorB.Target_Encoder *= s;
                }
            }
            if (MotorA.Target_Encoder < 0.0f) MotorA.Target_Encoder = 0.0f;
            if (MotorB.Target_Encoder < 0.0f) MotorB.Target_Encoder = 0.0f;
            if (++g_slow_timer >= 150) {
                g_white_count  = 0;
                g_straight_sub = STRAIGHT_SUB_DRIVE;
                g_task_state   = TASK_PHASE3_TRACK2;
            }
            break;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE3_TRACK2:
        Line_Tracking_Update(sensor_data);

        if (is_all_white(sensor_data)) {
            g_white_count++;
            if (g_white_count >= TASK_DEBOUNCE_COUNT) {
                g_target_yaw = g_initial_yaw;
                g_task_state = TASK_PHASE3_ALIGN0;
            }
        } else {
            g_white_count = 0;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE3_ALIGN0:
        if (JY62_Is_Data_Ready()) {
            yaw_error = normalize_180(g_target_yaw - JY62_Get_Yaw());

            if (yaw_error > -TASK_ALIGN_THRESHOLD && yaw_error < TASK_ALIGN_THRESHOLD) {
                g_phase3_loop++;
                if (g_phase3_loop < 4) {
                    g_target_yaw  = normalize_180(JY62_Get_Yaw() - ARCTAN_0_8_DEG);
                    Gyro_Hold_Set_Reference();
                    g_black_count = 0;
                    g_task_state  = TASK_PHASE3_RTURN_ALIGN;
                } else {
                    MotorA.Target_Encoder = 0.0f;
                    MotorB.Target_Encoder = 0.0f;
                    g_key_pending = 0;
                    g_task_state  = TASK_PHASE3_DONE;
                }
            } else if (yaw_error > 0) {
                MotorA.Target_Encoder = -TASK_ALIGN_SPEED;
                MotorB.Target_Encoder =  TASK_ALIGN_SPEED;
            } else {
                MotorA.Target_Encoder =  TASK_ALIGN_SPEED;
                MotorB.Target_Encoder = -TASK_ALIGN_SPEED;
            }
        } else {
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = 0.0f;
        }
        break;

    /* ============================================================== */
    case TASK_PHASE3_DONE:
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        break;
    }
}
