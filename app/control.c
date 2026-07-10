#include "control.h"

Encoder OriginalEncoder; 				// Encoder raw data
Motor_parameter MotorA,MotorB;			// Left/right motor parameters

float Velocity_KP=400, Velocity_KI=300;
PID_Parameter V_PID={400.0f,300.0f};	// PI parameters

/**************************************************************************
Function: Get_Velocity_From_Encoder
Input   : Encoder1 - encoder A raw count, Encoder2 - encoder B raw count
Output  : none
Description: Converts encoder raw counts to filtered speed (m/s)
  Uses first-order low-pass filter to smooth encoder noise
  raw speed = encoder_count * Frequency * Perimeter / (lines * 2x * gear_ratio)
**************************************************************************/
void Get_Velocity_From_Encoder(int Encoder1,int Encoder2)
{
	static float Filtered_SpeedA = 0.0f, Filtered_SpeedB = 0.0f;
	float Encoder_A_pr, Encoder_B_pr;
	float raw_speedA, raw_speedB;

	// Retrieve original encoder data
	OriginalEncoder.A = Encoder1;
	OriginalEncoder.B = Encoder2;
	Encoder_A_pr = OriginalEncoder.A;
	Encoder_B_pr = -OriginalEncoder.B;

	// Convert encoder counts to speed (m/s)
	raw_speedA = Encoder_A_pr * Frequency * Perimeter / (ENCODER_LINES * MULTIPLY_FACTOR * GEAR_RATIO);
	raw_speedB = Encoder_B_pr * Frequency * Perimeter / (ENCODER_LINES * MULTIPLY_FACTOR * GEAR_RATIO);

	// First-order low-pass filter: smooth encoder noise
	Filtered_SpeedA = SPEED_FILTER_ALPHA * raw_speedA + (1.0f - SPEED_FILTER_ALPHA) * Filtered_SpeedA;
	Filtered_SpeedB = SPEED_FILTER_ALPHA * raw_speedB + (1.0f - SPEED_FILTER_ALPHA) * Filtered_SpeedB;

	MotorA.Current_Encoder = Filtered_SpeedA;
	MotorB.Current_Encoder = Filtered_SpeedB;
}

/**************************************************************************
Function: Absolute value function
Input   : a - number to convert
Output  : absolute value
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0) temp=-a;
	else temp=a;
	return temp;
}

/**************************************************************************
Function: PWM_Limit
Input   : IN - input value, max - upper limit, min - lower limit
Output  : limited value
**************************************************************************/
float PWM_Limit(float IN, float max, float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}

/**************************************************************************
Function: Incremental PI controller (Left motor)
Input   : Encoder - current speed (m/s), Target - target speed (m/s)
Output  : PWM value
Description: Incremental discrete PI formula
  pwm += Kp*[e(k) - e(k-1)] + Ki*e(k)
  e(k) = Target - Encoder (current bias)
  Includes deadband: stops PI accumulation when bias is below threshold
  to prevent integral windup from encoder noise
**************************************************************************/
int Incremental_PI_Left (float Encoder, float Target)
{
	static float Bias, Pwm, Last_bias;
	float abs_bias;
	Bias = Target - Encoder;                	// Calculate bias

	// Deadband: when bias is small, stop PI accumulation to prevent windup
	abs_bias = (Bias > 0.0f) ? Bias : -Bias;
	if(abs_bias < PI_DEADBAND)
	{
		Last_bias = Bias;  // Still update bias for smooth recovery on exit
		return (int)Pwm;
	}

	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;   // Incremental PI
	Last_bias = Bias;	                   	// Save last bias
	Pwm = PWM_Limit(Pwm, PWM_MAX, -PWM_MAX);
	return (int)Pwm;
}

/**************************************************************************
Function: Incremental PI controller (Right motor)
Input   : Encoder - current speed (m/s), Target - target speed (m/s)
Output  : PWM value
**************************************************************************/
int Incremental_PI_Right (float Encoder, float Target)
{
	static float Bias, Pwm, Last_bias;
	float abs_bias;
	Bias = Target - Encoder;                	// Calculate bias

	// Deadband: when bias is small, stop PI accumulation to prevent windup
	abs_bias = (Bias > 0.0f) ? Bias : -Bias;
	if(abs_bias < PI_DEADBAND)
	{
		Last_bias = Bias;  // Still update bias for smooth recovery on exit
		return (int)Pwm;
	}

	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;   // Incremental PI
	Last_bias = Bias;	                   	// Save last bias
	Pwm = PWM_Limit(Pwm, PWM_MAX, -PWM_MAX);
	return (int)Pwm;
}
