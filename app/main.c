#include "board.h"
#include "oled.h"
#include "gyro_hold.h"

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
	JY62_Init();
	Gyro_Hold_Init();

	NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
	DL_UART_Main_disableLoopbackMode(UART_1_INST);
	DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);

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
		static uint8_t display_page = 0;
		display_page = !display_page;

		if (display_page)
		{
			/* Page 0: Speed + Status + Sensor */
			OLED_ShowString(0, 0, (const uint8_t *)"MA_V:");
			OLED_ShowNumber(40, 0, (uint32_t)(MotorA.Current_Encoder * 100), 4, 12);
			OLED_ShowString(0, 20, (const uint8_t *)"MB_V:");
			OLED_ShowNumber(40, 20, (uint32_t)(MotorB.Current_Encoder * 100), 4, 12);
			OLED_ShowString(0, 40, (const uint8_t *)"Status:");
			OLED_ShowString(60, 40, Flag_Stop ? (const uint8_t *)"STOP" : (const uint8_t *)"RUN ");

			{
				uint8_t i;
				for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
					OLED_ShowNumber(i * 16, 52, g_sensor_data[i], 1, 12);
				}
			}
		}
		else
		{
			/* Page 1: Gyro debug info */
			int wz_int  = (int)(JY62_Get_AngularVelocityZ() * 10.0f);
			int yaw_int = (int)(JY62_Get_Yaw() * 10.0f);

			OLED_ShowString(0, 0, (const uint8_t *)"Gyro Debug");
			OLED_ShowString(0, 20, (const uint8_t *)"wz:");
			if (wz_int < 0) {
				OLED_ShowString(24, 20, (const uint8_t *)"-");
				OLED_ShowNumber(32, 20, (uint32_t)(-wz_int), 4, 12);
			} else {
				OLED_ShowNumber(24, 20, (uint32_t)(wz_int), 4, 12);
			}

			OLED_ShowString(0, 40, (const uint8_t *)"yaw:");
			if (yaw_int < 0) {
				OLED_ShowString(32, 40, (const uint8_t *)"-");
				OLED_ShowNumber(40, 40, (uint32_t)(-yaw_int), 4, 12);
			} else {
				OLED_ShowNumber(32, 40, (uint32_t)(yaw_int), 4, 12);
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

// UART1 RX interrupt for JY62 gyroscope
void UART_1_INST_IRQHandler(void)
{
	switch (DL_UART_getPendingInterrupt(UART_1_INST)) {
		case DL_UART_IIDX_RX:
			JY62_UART_RX_ISR(DL_UART_Main_receiveData(UART_1_INST));
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
