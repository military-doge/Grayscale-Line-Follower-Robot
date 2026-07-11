#include "line_track.h"
#include "control.h"

int16_t g_pid_output = 0;
int8_t  g_line_error  = 0;

static float  g_integral   = 0.0f;
static int8_t g_error_last = 0;

static float Line_PID_Calc(int8_t error)
{
	float output;

	g_integral += error;

	output = error * LINE_TRACK_KP
	       + LINE_TRACK_KI * g_integral
	       + (error - g_error_last) * LINE_TRACK_KD;

	g_error_last = error;

	if (output > LINE_PID_MAX + 200.0f)
		output = LINE_PID_MAX + 200.0f;
	if (output < -LINE_PID_MAX - 200.0f)
		output = -LINE_PID_MAX - 200.0f;

	return output;
}

void Line_Tracking_Init(void)
{
	g_pid_output = 0;
	g_line_error = 0;
	g_integral   = 0.0f;
	g_error_last = 0;
}

void Line_Tracking_Update(uint16_t *sensor_data)
{
	static int8_t err = 0;
	int8_t  i;
	int8_t  active_count = 0;
	int16_t weighted_sum = 0;
	float   pid_out, turn_speed;

	/*
	 * Weighted-average algorithm:
	 *   Weights map sensor position to error:
	 *     X1=-30  X2=-20  X3=-15  X4=-5   X5=+5   X6=+15  X7=+20  X8=+30
	 *   err = sum(weight[i] * sensor[i]) / active_count
	 *   Continuous output, no discrete jumps.
	 */
	static const int8_t weights[8] = {-30, -20, -15, -5, 5, 15, 20, 30};

	for (i = 0; i < 8; i++)
	{
		if (sensor_data[i] == ACTIVE_LEVEL)
		{
			weighted_sum   += weights[i];
			active_count++;
		}
	}

	if (active_count > 0)
	{
		err = weighted_sum / active_count;
	}
	else
	{
		/* Line lost: search in last known direction */
		if (err > 0)
			err = 30;
		else if (err < 0)
			err = -30;
	}

	g_line_error = err;

	/* Low-pass filter + quadratic error scaling */
	{
		static float filtered_err  = 0.0f;
		float abs_err, nonlinear_err;

		filtered_err = ERROR_FILTER_ALPHA * (float)err
		             + (1.0f - ERROR_FILTER_ALPHA) * filtered_err;

		/* quadratic: small errors suppressed, large errors amplified */
		abs_err = (filtered_err >= 0.0f) ? filtered_err : -filtered_err;
		nonlinear_err = filtered_err * abs_err / 5.0f;

		pid_out = Line_PID_Calc((int8_t)nonlinear_err);
	}

	g_pid_output = (int16_t)pid_out;

	turn_speed = pid_out * STEERING_GAIN;

	MotorA.Target_Encoder = LINE_BASE_SPEED + turn_speed;
	MotorB.Target_Encoder = LINE_BASE_SPEED - turn_speed;
}
