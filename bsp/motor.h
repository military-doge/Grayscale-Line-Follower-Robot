#ifndef _MOTOR_H
#define _MOTOR_H
#include "ti_msp_dl_config.h"
#include "board.h"

void Set_PWM(int pwmA,int pwmB);
int limit_PWM(int value,int low,int high);
#endif
