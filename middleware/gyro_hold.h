#ifndef _GYRO_HOLD_H
#define _GYRO_HOLD_H

#include <stdint.h>

void  Gyro_Hold_Init(void);
void  Gyro_Hold_Set_Reference(void);
void  Gyro_Hold_Clear(void);
float Gyro_Hold_Get_Correction(void);
float Gyro_Hold_Get_Error(void);

#endif
