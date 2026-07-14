#ifndef _GYRO_HOLD_H
#define _GYRO_HOLD_H

#include <stdint.h>

void Gyro_Hold_Init(void);
void Gyro_Hold_Set_Reference(void);
void Gyro_Hold_Clear(void);
float Gyro_Hold_Get_Correction(void);
float Gyro_Hold_Get_Error(void);

/* Gyro-straight sequence (3 modes cycle: simple → complex → pattern ×4 → ...) */
void    Gyro_Straight_Start(void);
uint8_t Gyro_Straight_Update(uint16_t *sensor_data);
uint8_t Gyro_Straight_Is_Active(void);

#endif
