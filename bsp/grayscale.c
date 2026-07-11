#include "grayscale.h"
#include "board.h"

static void _select_channel(uint8_t channel)
{
	uint8_t ad0 = (channel >> 0) & 1;
	uint8_t ad1 = (channel >> 1) & 1;
	uint8_t ad2 = (channel >> 2) & 1;

	if (ad0)
		DL_GPIO_setPins(GS_AD_PORT, GS_AD_AD0_PIN);
	else
		DL_GPIO_clearPins(GS_AD_PORT, GS_AD_AD0_PIN);

	if (ad1)
		DL_GPIO_setPins(GS_AD_PORT, GS_AD_AD1_PIN);
	else
		DL_GPIO_clearPins(GS_AD_PORT, GS_AD_AD1_PIN);

	if (ad2)
		DL_GPIO_setPins(GS_IO_PORT, GS_IO_AD2_PIN);
	else
		DL_GPIO_clearPins(GS_IO_PORT, GS_IO_AD2_PIN);
}

static uint16_t Read_OUT_value(void)
{
	return !!(DL_GPIO_readPins(GS_IO_PORT, GS_IO_OUT_PIN));
}

void Grayscale_Sensor_Init(void)
{
}

void Grayscale_Sensor_Read_All(uint16_t *sensor_values)
{
	uint8_t i;

	for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
	{
		_select_channel(i);
		delay_us(50);
		sensor_values[i] = Read_OUT_value();
	}
}

uint16_t Grayscale_Sensor_Read_Single(uint8_t channel)
{
	if (channel >= GRAYSCALE_SENSOR_CHANNELS)
		return 0;

	_select_channel(channel);
	delay_us(50);
	return Read_OUT_value();
}
