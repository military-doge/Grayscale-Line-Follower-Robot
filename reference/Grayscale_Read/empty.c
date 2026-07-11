//Grayscale Sensor / MSPM0
//		5V			5V
//		GND			GND
//		AD0			PA14
//		AD1			PA15
//		AD2			PA16
//		OUT			PA17
#include "ti_msp_dl_config.h"
#include "delay.h"
#include "usart.h"
#include "grayscale_sensor.h"

uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
int i;
const char* sensor_labels[GRAYSCALE_SENSOR_CHANNELS] = {
        "X1", "X2", "X3", "X4", "X5", "X6", "X7", "X8"
    };

int main(void)
{
    // SYSCFG初始化
	// SYSCFG initialization
	SYSCFG_DL_init();
	USART_Init();

 	while(1)
	{		
		Grayscale_Sensor_Read_All(g_sensor_data);
        
        for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
        {
            printf("[ %s:%d ]", sensor_labels[i], g_sensor_data[i]);
        }
        printf("\n");
		//当X1的灯亮起时，值为1 / When the X1 light is on, the value is 1
		delay_ms(30);
	}	 
}



