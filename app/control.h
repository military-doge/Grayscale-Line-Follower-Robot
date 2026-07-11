#ifndef __CONTROL_H
#define __CONTROL_H

#include "board.h"

#define Frequency	100.0f			// 10ms sample period = 100Hz
#define Perimeter	0.14765f        // Wheel circumference (m) = 47mm * PI
#define Wheelspacing 0.13f          // Wheel spacing (m)
#define PI 3.1415926

// Speed low-pass filter coefficient (0~1, lower = stronger filtering, 0.3~0.5 recommended)
#define SPEED_FILTER_ALPHA  0.4f
// PI controller deadband (m/s), below this threshold Pwm stops accumulating
#define PI_DEADBAND         0.005f
// PWM output limit (max 8000, 7800 = 97.5% duty cycle, avoids full saturation)
#define PWM_MAX             7800

// Encoder parameters (adjust to match actual hardware)
#define ENCODER_LINES  	13       // Encoder lines per revolution
#define MULTIPLY_FACTOR 2       // 2x quadrature (both edges)
#define GEAR_RATIO   	28      // Gear ratio 28:1

// Encoder raw data struct
typedef struct
{
  int A;
  int B;
}Encoder;

// Motor speed control parameter struct
typedef struct
{
	float Current_Encoder;     	// Current speed from encoder (filtered, m/s)
	float Motor_Pwm;     		// Output PWM value (control output)
	float Target_Encoder;  		// Target speed setpoint (m/s)
	float Velocity; 	 		// Motor velocity
}Motor_parameter;

typedef struct
{
	float Kp;
	float ki;
}PID_Parameter;

extern Encoder OriginalEncoder;
extern Motor_parameter MotorA,MotorB;
extern PID_Parameter V_PID;
extern float Velocity_KP,Velocity_KI;

void Get_Velocity_From_Encoder(int Encoder1,int Encoder2);
int myabs(int a);
int Incremental_PI_Left (float Encoder,float Target);
int Incremental_PI_Right (float Encoder,float Target);

#endif
