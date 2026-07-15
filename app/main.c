#include "board.h"
#include "oled.h"
#include "gyro_hold.h"
#include "jy62.h"
#include "dma_rx.h"

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

			OLED_ShowString(0, 2, (const uint8_t *)"Yaw");
			if (offset_val < 0) {
				OLED_ShowString(30, 2, (const uint8_t *)"-");
				OLED_ShowNumber(38, 2, (uint32_t)(-offset_val), 4, 12);
			} else {
				OLED_ShowNumber(30, 2, (uint32_t)(offset_val), 4, 12);
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
			static int prev_stop = 1;
			Grayscale_Sensor_Read_All(g_sensor_data);
			DMA_RX_Process();
			// LED_Flash(100);
			{
				KeyEvent ke = Key();
				if (ke == KEY_EVENT_CLICK) {
					if (Flag_Stop) {
						Gyro_Straight_Start();
						Flag_Stop = 0;
					} else {
						Flag_Stop = 1;
						MotorA.Target_Encoder = MotorB.Target_Encoder = 0.0f;
					}
				}
			}
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
