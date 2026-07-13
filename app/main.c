#include "board.h"
#include "oled.h"
#include <string.h>

int Flag_Stop = 1;
volatile uint32_t g_tick_10ms = 0;
uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

void user_init(void)
{
	Velocity_KP = V_PID.Kp;
	Velocity_KI = V_PID.ki;

	DL_Timer_startCounter(PWM_0_INST);
	NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
	NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
	NVIC_EnableIRQ(ENCODERA_INT_IRQN);
	NVIC_EnableIRQ(ENCODERB_INT_IRQN);
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

	Grayscale_Sensor_Init();
	Line_Tracking_Init();

	Flag_Stop = 1;
	MotorA.Target_Encoder = MotorB.Target_Encoder = 0.0f;
}

void user_main(void)
{
	static uint32_t last_display_tick = 0;

	while (1)
	{
		// Update OLED display every 100ms (10 * 10ms)
		if (g_tick_10ms - last_display_tick >= 10)
		{
			memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8)); // GRAM清零，防止残影

			// Line 1: Mode + Error
			if (Flag_Stop)
				OLED_ShowString(0, 0, (const uint8_t *)"STOP ");
			else
			{
				OLED_ShowString(0, 0, (const uint8_t *)"TRACK");
				OLED_ShowString(52, 0, (const uint8_t *)"E:");
				OLED_ShowNumber(68, 0, (int)Tracking_Get_Last_Error(), 4, 10);
			}

			// Line 2: PWM L/R
			OLED_ShowString(0, 10, (const uint8_t *)"PWM");
			OLED_ShowString(30, 10, (const uint8_t *)"L:");
			OLED_ShowNumber(46, 10, myabs((int)(MotorA.Motor_Pwm)), 4, 12);
			OLED_ShowString(82, 10, (const uint8_t *)"R:");
			OLED_ShowNumber(98, 10, myabs((int)(MotorB.Motor_Pwm)), 4, 12);

			// Line 3: Left motor target + current speed (mm/s)
			OLED_ShowString(0, 20, (const uint8_t *)"L");
			if ((MotorA.Target_Encoder * 1000) < 0)
				OLED_ShowString(16, 20, (const uint8_t *)"-"),
				OLED_ShowNumber(26, 20, myabs((int)(MotorA.Target_Encoder * 1000)), 4, 12);
			else
				OLED_ShowString(16, 20, (const uint8_t *)"+"),
				OLED_ShowNumber(26, 20, myabs((int)(MotorA.Target_Encoder * 1000)), 4, 12);

			if (MotorA.Current_Encoder < 0)
				OLED_ShowString(60, 20, (const uint8_t *)"-");
			else
				OLED_ShowString(60, 20, (const uint8_t *)"+");
			OLED_ShowNumber(68, 20, myabs((int)(MotorA.Current_Encoder * 1000)), 4, 12);
			OLED_ShowString(96, 20, (const uint8_t *)"mm/s");

			// Line 4: Right motor target + current speed (mm/s)
			OLED_ShowString(0, 30, (const uint8_t *)"R");
			if ((MotorB.Target_Encoder * 1000) < 0)
				OLED_ShowString(16, 30, (const uint8_t *)"-"),
				OLED_ShowNumber(26, 30, myabs((int)(MotorB.Target_Encoder * 1000)), 4, 12);
			else
				OLED_ShowString(16, 30, (const uint8_t *)"+"),
				OLED_ShowNumber(26, 30, myabs((int)(MotorB.Target_Encoder * 1000)), 4, 12);

			if (MotorB.Current_Encoder < 0)
				OLED_ShowString(60, 30, (const uint8_t *)"-");
			else
				OLED_ShowString(60, 30, (const uint8_t *)"+");
			OLED_ShowNumber(68, 30, myabs((int)(MotorB.Current_Encoder * 1000)), 4, 12);
			OLED_ShowString(96, 30, (const uint8_t *)"mm/s");

			// Line 5: Sensor values
			{
				uint8_t i;
				for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
					OLED_ShowNumber(i * 16, 42, g_sensor_data[i], 1, 12);
				}
			}
			OLED_Refresh_Gram();
			last_display_tick = g_tick_10ms;
		}
	}
}

int main(void)
{
	SYSCFG_DL_init();
	OLED_Init();
	OLED_ShowString(0, 0, (const uint8_t *)"GRAYSCALE ROBOT");
	OLED_ShowString(0, 20, (const uint8_t *)"Init OK");
	OLED_Refresh_Gram();
	delay_ms(500);

	user_init();
	while (1)
	{
		user_main();
	}
}

// 10ms timer interrupt
void TIMER_0_INST_IRQHandler(void)
{
	g_tick_10ms++;

	switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
		case DL_TIMERG_IIDX_ZERO:
		{
			static int prev_stop = 1;
			Grayscale_Sensor_Read_All(g_sensor_data);
			LED_Flash(100);
			Key();
			Get_Velocity_From_Encoder(Get_Encoder_countA, Get_Encoder_countB);
			Get_Encoder_countA = Get_Encoder_countB = 0;

			if (Flag_Stop) {
				MotorA.Target_Encoder = 0.0f;
				MotorB.Target_Encoder = 0.0f;
				prev_stop = 1;
			} else {
				if (prev_stop) {
					Tracking_Reset_PID();  // 启动时复位PID状态
					prev_stop = 0;
				}
				Line_Tracking_Update(g_sensor_data);
			}

			// Incremental PI control for left/right motors
			MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder, MotorA.Target_Encoder);
			MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder, MotorB.Target_Encoder);
			Set_PWM(MotorA.Motor_Pwm, MotorB.Motor_Pwm);
			break;
		}
		default:
			break;
	}
}

// External interrupt for encoder reading
void GROUP1_IRQHandler(void)
{
	Get_Encoder();
}
