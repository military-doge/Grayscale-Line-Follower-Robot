#include "board.h"
#include "oled.h"

int Flag_Stop = 1;

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

	Flag_Stop = 1;
	MotorA.Target_Encoder = MotorB.Target_Encoder = 0.0f;
}

void user_main(void)
{
	static uint32_t last_display_tick = 0;
	uint32_t now;

	while (1)
	{
		now = Systick_getTick();

		// Update OLED display every 500ms
		if (((now - last_display_tick) & SysTickMAX_COUNT) >= SysTick_MS(500))
		{
			OLED_ShowString(0, 0, (const uint8_t *)"MA_V:");
			OLED_ShowNumber(40, 0, (uint32_t)(MotorA.Current_Encoder * 100), 4, 12);
			OLED_ShowString(0, 20, (const uint8_t *)"MB_V:");
			OLED_ShowNumber(40, 20, (uint32_t)(MotorB.Current_Encoder * 100), 4, 12);
			OLED_ShowString(0, 40, (const uint8_t *)"Status:");
			OLED_ShowString(60, 40, Flag_Stop ? (const uint8_t *)"STOP" : (const uint8_t *)"RUN ");
			OLED_Refresh_Gram();
			last_display_tick = now;
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
	switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
		case DL_TIMERG_IIDX_ZERO:
			LED_Flash(100);
			Key();
			Get_Velocity_From_Encoder(Get_Encoder_countA, Get_Encoder_countB);
			Get_Encoder_countA = Get_Encoder_countB = 0;
			// Incremental PI control for left/right motors
			MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder, MotorA.Target_Encoder);
			MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder, MotorB.Target_Encoder);
			Set_PWM(MotorA.Motor_Pwm, MotorB.Motor_Pwm);
			break;
		default:
			break;
	}
}

// External interrupt for encoder reading
void GROUP1_IRQHandler(void)
{
	Get_Encoder();
}
