#ifndef _LINE_TRACK_H
#define _LINE_TRACK_H

#include <stdint.h>

#define LINE_TRACK_KP     30.0f
#define LINE_TRACK_KI     0.0f
#define LINE_TRACK_KD     100.0f

#define ACTIVE_LEVEL         1
#define ERROR_FILTER_ALPHA   0.5f   /* low-pass filter for error smoothing */

#define LINE_BASE_SPEED   0.10f
#define STEERING_GAIN     0.000065f    /* half_wheelbase / 1000 = 0.065 / 1000 */
#define LINE_PID_MAX      1200.0f

#define SHARP_TURN_SPEED  0.02f

extern int16_t g_pid_output;
extern int8_t  g_line_error;

void Line_Tracking_Init(void);
void Line_Tracking_Update(uint16_t *sensor_data);

#endif /* _LINE_TRACK_H */
