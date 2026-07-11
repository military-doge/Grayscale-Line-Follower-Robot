#include "line_track.h"
#include "control.h"

int16_t g_pid_output = 0;
int8_t  g_line_error  = 0;

static uint16_t g_sharp_turn_ticks = 0;
static uint8_t  g_sharp_turn_dir   = 0;  /* 0=none, 1=left, 2=right */
static float    g_integral         = 0.0f;
static int8_t   g_error_last       = 0;

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
	g_sharp_turn_ticks = 0;
	g_sharp_turn_dir   = 0;
	g_integral   = 0.0f;
	g_error_last = 0;
}

void Line_Tracking_Update(uint16_t *sensor_data)
{
	uint8_t x1, x2, x3, x4, x5, x6, x7, x8;
	int8_t  err = 0;
	float   pid_out;
	float   base_speed, turn_speed;

	x1 = sensor_data[0];
	x2 = sensor_data[1];
	x3 = sensor_data[2];
	x4 = sensor_data[3];
	x5 = sensor_data[4];
	x6 = sensor_data[5];
	x7 = sensor_data[6];
	x8 = sensor_data[7];

	/* Sharp turn timeout handling (non-blocking via tick counter) */
	if (g_sharp_turn_ticks > 0)
	{
		g_sharp_turn_ticks--;
		if (g_sharp_turn_dir == 1)
		{
			MotorA.Target_Encoder = -SHARP_TURN_SPEED;
			MotorB.Target_Encoder = -SHARP_TURN_SPEED;
		}
		else if (g_sharp_turn_dir == 2)
		{
			MotorA.Target_Encoder = -SHARP_TURN_SPEED;
			MotorB.Target_Encoder = -SHARP_TURN_SPEED;
		}
		return;
	}

	/* ---- Pattern matching from reference ---- */

	/* 1. Dead centre: X4 and X5 on line, edges clear */
	if (x4 == ACTIVE_LEVEL && x5 == ACTIVE_LEVEL &&
	    x1 != ACTIVE_LEVEL && x8 != ACTIVE_LEVEL)
	{
		err = 0;
	}
	/* 2. Sharp left turn: left side + centre */
	else if ((x1 == ACTIVE_LEVEL || x2 == ACTIVE_LEVEL) &&
	         (x4 == ACTIVE_LEVEL || x5 == ACTIVE_LEVEL))
	{
		err = -20;
		g_pid_output = (int16_t)Line_PID_Calc(err);
		MotorA.Target_Encoder = -SHARP_TURN_SPEED;
		MotorB.Target_Encoder = -SHARP_TURN_SPEED;
		g_sharp_turn_ticks = 30;  /* 300ms @ 10ms tick */
		g_sharp_turn_dir   = 1;
		return;
	}
	/* 3. Sharp right turn: right side + centre */
	else if ((x7 == ACTIVE_LEVEL || x8 == ACTIVE_LEVEL) &&
	         (x4 == ACTIVE_LEVEL || x5 == ACTIVE_LEVEL))
	{
		err = 20;
		g_pid_output = (int16_t)Line_PID_Calc(err);
		MotorA.Target_Encoder = -SHARP_TURN_SPEED;
		MotorB.Target_Encoder = -SHARP_TURN_SPEED;
		g_sharp_turn_ticks = 30;  /* 300ms @ 10ms tick */
		g_sharp_turn_dir   = 2;
		return;
	}
	/* 4. Slightly left: X4 on, X5 off */
	else if (x4 == ACTIVE_LEVEL && x5 != ACTIVE_LEVEL)
	{
		err = -1;
	}
	/* 5. Slightly right: X4 off, X5 on */
	else if (x4 != ACTIVE_LEVEL && x5 == ACTIVE_LEVEL)
	{
		err = 1;
	}
	/* 6. Left shift patterns */
	else if (x1 != ACTIVE_LEVEL && x2 != ACTIVE_LEVEL && x3 != ACTIVE_LEVEL &&
	         x4 == ACTIVE_LEVEL && x5 != ACTIVE_LEVEL &&
	         x6 != ACTIVE_LEVEL && x7 != ACTIVE_LEVEL && x8 != ACTIVE_LEVEL)
	{
		err = -1;
	}
	else if (x1 != ACTIVE_LEVEL && x2 != ACTIVE_LEVEL && x3 == ACTIVE_LEVEL &&
	         x4 == ACTIVE_LEVEL && x5 != ACTIVE_LEVEL &&
	         x6 != ACTIVE_LEVEL && x7 != ACTIVE_LEVEL && x8 != ACTIVE_LEVEL)
	{
		err = -2;
	}
	else if (x1 != ACTIVE_LEVEL && x2 != ACTIVE_LEVEL && x3 == ACTIVE_LEVEL &&
	         x4 != ACTIVE_LEVEL && x5 != ACTIVE_LEVEL &&
	         x6 != ACTIVE_LEVEL && x7 != ACTIVE_LEVEL && x8 != ACTIVE_LEVEL)
	{
		err = -4;
	}
	else if (x1 != ACTIVE_LEVEL && x2 == ACTIVE_LEVEL && x3 != ACTIVE_LEVEL &&
	         x4 != ACTIVE_LEVEL && x5 != ACTIVE_LEVEL &&
	         x6 != ACTIVE_LEVEL && x7 != ACTIVE_LEVEL && x8 != ACTIVE_LEVEL)
	{
		err = -6;
	}
	else if (x1 == ACTIVE_LEVEL && x2 != ACTIVE_LEVEL && x3 != ACTIVE_LEVEL &&
	         x4 != ACTIVE_LEVEL && x5 != ACTIVE_LEVEL &&
	         x6 != ACTIVE_LEVEL && x7 != ACTIVE_LEVEL && x8 != ACTIVE_LEVEL)
	{
		err    = -12;
		base_speed = LINE_BASE_SPEED - (70.0f * STEERING_GAIN);
	}
	/* 7. Right shift patterns */
	else if (x1 != ACTIVE_LEVEL && x2 != ACTIVE_LEVEL && x3 != ACTIVE_LEVEL &&
	         x4 != ACTIVE_LEVEL && x5 == ACTIVE_LEVEL && x6 == ACTIVE_LEVEL &&
	         x7 != ACTIVE_LEVEL && x8 != ACTIVE_LEVEL)
	{
		err = 3;
	}
	else if (x1 != ACTIVE_LEVEL && x2 != ACTIVE_LEVEL && x3 != ACTIVE_LEVEL &&
	         x4 != ACTIVE_LEVEL && x5 != ACTIVE_LEVEL && x6 == ACTIVE_LEVEL &&
	         x7 != ACTIVE_LEVEL && x8 != ACTIVE_LEVEL)
	{
		err = 5;
	}
	else if (x1 != ACTIVE_LEVEL && x2 != ACTIVE_LEVEL && x3 != ACTIVE_LEVEL &&
	         x4 != ACTIVE_LEVEL && x5 != ACTIVE_LEVEL && x6 == ACTIVE_LEVEL &&
	         x7 == ACTIVE_LEVEL && x8 != ACTIVE_LEVEL)
	{
		err = 8;
	}
	else if (x1 != ACTIVE_LEVEL && x2 != ACTIVE_LEVEL && x3 != ACTIVE_LEVEL &&
	         x4 != ACTIVE_LEVEL && x5 != ACTIVE_LEVEL && x6 != ACTIVE_LEVEL &&
	         x7 != ACTIVE_LEVEL && x8 == ACTIVE_LEVEL)
	{
		err    = 12;
		base_speed = LINE_BASE_SPEED - (70.0f * STEERING_GAIN);
	}
	/* 8. Multi-sensor sharp left */
	else if (x1 == ACTIVE_LEVEL && x2 == ACTIVE_LEVEL &&
	         (x3 == ACTIVE_LEVEL || x4 == ACTIVE_LEVEL || x5 == ACTIVE_LEVEL) &&
	         x8 != ACTIVE_LEVEL)
	{
		err = -15;
		g_pid_output = (int16_t)Line_PID_Calc(err);
		MotorA.Target_Encoder = -SHARP_TURN_SPEED;
		MotorB.Target_Encoder = -SHARP_TURN_SPEED;
		g_sharp_turn_ticks = 25;  /* 250ms */
		g_sharp_turn_dir   = 1;
		return;
	}
	/* 9. Multi-sensor sharp right */
	else if (x7 == ACTIVE_LEVEL && x8 == ACTIVE_LEVEL &&
	         (x6 == ACTIVE_LEVEL || x4 == ACTIVE_LEVEL || x5 == ACTIVE_LEVEL) &&
	         x1 != ACTIVE_LEVEL)
	{
		err = 15;
		g_pid_output = (int16_t)Line_PID_Calc(err);
		MotorA.Target_Encoder = -SHARP_TURN_SPEED;
		MotorB.Target_Encoder = -SHARP_TURN_SPEED;
		g_sharp_turn_ticks = 30;  /* 300ms */
		g_sharp_turn_dir   = 2;
		return;
	}

	/* Apply PID and set motor targets */
	g_line_error = err;
	base_speed   = LINE_BASE_SPEED;

	pid_out    = Line_PID_Calc(err);
	g_pid_output = (int16_t)pid_out;

	turn_speed = pid_out * STEERING_GAIN;

	MotorA.Target_Encoder = base_speed + turn_speed;
	MotorB.Target_Encoder = base_speed - turn_speed;
}
