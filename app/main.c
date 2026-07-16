#include "board.h"
#include "oled.h"
#include "gyro_hold.h"
#include "jy62.h"
#include "dma_rx.h"
#include "task_planner.h"

int Flag_Stop = 1;
volatile uint32_t g_tick_10ms = 0;
volatile uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

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
	DL_TimerG_startCounter(TIMER_0_INST);

	Grayscale_Sensor_Init();
	Line_Tracking_Init();
	JY62_Init();
	Gyro_Hold_Init();
	Task_Planner_Init();

	DL_UART_Main_disable(UART_2_INST);
	DL_UART_Main_disableLoopbackMode(UART_2_INST);
	DL_UART_Main_enable(UART_2_INST);

	/* Flush any stale data in RX FIFO before starting DMA */
	while (!DL_UART_isRXFIFOEmpty(UART_2_INST)) {
		DL_UART_receiveData(UART_2_INST);
	}

	DMA_RX_Init(JY62_UART_RX_ISR);

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
			int offset_val = (int)(Gyro_Hold_Get_Error());
			TaskState st = Task_Planner_Get_State();

			{
				uint8_t i, cnt = 0;
				for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
					if (g_sensor_data[i]) cnt++;
				}

				OLED_ShowString(0, 0, (const uint8_t *)"S:");
				OLED_ShowNumber(16, 0, (uint32_t)st, 1, 12);
				OLED_ShowString(30, 0, (const uint8_t *)"C:");
				OLED_ShowNumber(46, 0, cnt, 1, 12);
				OLED_ShowString(60, 0, (const uint8_t *)"Er:");
				if (g_line_error < 0) {
					OLED_ShowString(84, 0, (const uint8_t *)"-");
					OLED_ShowNumber(92, 0, (uint32_t)(-g_line_error), 2, 12);
				} else {
					OLED_ShowNumber(84, 0, (uint32_t)(g_line_error), 2, 12);
				}
			}

			OLED_ShowString(0, 16, (const uint8_t *)"Yaw");
			if (offset_val < 0) {
				OLED_ShowString(30, 16, (const uint8_t *)"-");
				OLED_ShowNumber(38, 16, (uint32_t)(-offset_val), 4, 12);
			} else {
				OLED_ShowNumber(30, 16, (uint32_t)(offset_val), 4, 12);
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
	OLED_Clear();
	OLED_Refresh_Gram();

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
			Grayscale_Sensor_Read_All(g_sensor_data);
			DMA_RX_Process();
			// LED_Flash(100);
			{
				KeyEvent ke = Key();
				if (ke == KEY_EVENT_CLICK) {
					Task_Planner_On_Key();
				}
			}
			Get_Velocity_From_Encoder(Get_Encoder_countA, Get_Encoder_countB);
			Get_Encoder_countA = Get_Encoder_countB = 0;

			Task_Planner_Update(g_sensor_data);

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

// UART2 interrupt — RX handled by DMA, keep stub
void UART_2_INST_IRQHandler(void)
{
	switch (DL_UART_getPendingInterrupt(UART_2_INST)) {
		case DL_UART_IIDX_RX:
			/* DMA handles data transfer */
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
