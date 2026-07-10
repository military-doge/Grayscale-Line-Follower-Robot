#include "motor.h"

int limit_PWM(int value,int low,int high)
{
	if(value>high) return high;
	else if(value<low) return low;
	else return value;
}

void Set_PWM(int pwmA,int pwmB)
{
	 if(pwmA>0)
    {
		DL_GPIO_setPins(AIN_PORT,AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT,AIN_AIN2_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmA),GPIO_PWM_0_C0_IDX);
    }
    else
    {
        DL_GPIO_setPins(AIN_PORT,AIN_AIN2_PIN);
        DL_GPIO_clearPins(AIN_PORT,AIN_AIN1_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmA),GPIO_PWM_0_C0_IDX);
    }
    if(pwmB>0)
    {
		DL_GPIO_setPins(BIN_PORT,BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmB),GPIO_PWM_0_C1_IDX);
    }
    else
    {
		DL_GPIO_setPins(BIN_PORT,BIN_BIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN1_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmB),GPIO_PWM_0_C1_IDX);
    }
}
