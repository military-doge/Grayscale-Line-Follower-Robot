#include "grayscale.h"
#include "board.h"

static void _select_channel(uint8_t channel)
{
	uint8_t ad0 = (channel >> 0) & 1;
	uint8_t ad1 = (channel >> 1) & 1;
	uint8_t ad2 = (channel >> 2) & 1;

	if (ad0)
		DL_GPIO_setPins(GS_PORT, GS_AD0_PIN);
	else
		DL_GPIO_clearPins(GS_PORT, GS_AD0_PIN);

	if (ad1)
		DL_GPIO_setPins(GS_PORT, GS_AD1_PIN);
	else
		DL_GPIO_clearPins(GS_PORT, GS_AD1_PIN);

	if (ad2)
		DL_GPIO_setPins(GS_PORT, GS_AD2_PIN);
	else
		DL_GPIO_clearPins(GS_PORT, GS_AD2_PIN);
}

static uint16_t Read_OUT_value(void)
{
	return !!(DL_GPIO_readPins(GS_PORT, GS_OUT_PIN));
}

void Grayscale_Sensor_Init(void)
{
	/*
	 * Configure the pins that differ from the (incorrect) SysConfig:
	 *
	 *   PA9  (AD0) — digital output, not in SysConfig at all
	 *   PA8  (AD1) — digital output, not in SysConfig at all
	 *   PA12 (AD2) — already output from SysConfig (GS_AD_AD1), ok as-is
	 *   PA27 (OUT) — SysConfig made this output (GS_AD_AD0); override to input
	 *
	 * The SysConfig GS_AD group already calls:
	 *   DL_GPIO_initDigitalOutput(GS_AD_AD0_IOMUX)  → PA27 (must override!)
	 *   DL_GPIO_initDigitalOutput(GS_AD_AD1_IOMUX)  → PA12 (already output, ok)
	 *   DL_GPIO_enableOutput(GPIOA, PA27 | PA12)
	 *
	 * The SysConfig GS_IO group calls:
	 *   DL_GPIO_initDigitalOutput(GS_IO_AD2_IOMUX)  → PB4  (wrong port!)
	 *   DL_GPIO_initDigitalInput(GS_IO_OUT_IOMUX)   → PB17 (wrong pin!)
	 *
	 * We fix everything here so the hardware matches.
	 */

	/* PA9  = AD0: digital output (not in SysConfig) */
	DL_GPIO_initDigitalOutput(GS_AD0_IOMUX);
	DL_GPIO_enableOutput(GS_PORT, GS_AD0_PIN);

	/* PA8  = AD1: digital output (not in SysConfig) */
	DL_GPIO_initDigitalOutput(GS_AD1_IOMUX);
	DL_GPIO_enableOutput(GS_PORT, GS_AD1_PIN);

	/* PA12 = AD2: already configured as output by SysConfig (GS_AD_AD1).
	 * But SysConfig also calls enableOutput on GPIOA for AD1=PA12 together
	 * with AD0=PA27, so the enableOutput side is already done.  The
	 * DL_GPIO_initDigitalOutput call in SysConfig used GS_AD_AD1_IOMUX
	 * which is IOMUX_PINCM34 — same pin, correct.  Nothing to do. */

	/* PA27 = OUT: override SysConfig — must be INPUT, not output.
	 * SysConfig enabled the output driver on this pin; turn it off
	 * so we don't fight the sensor signal. */
	DL_GPIO_disableOutput(GS_PORT, GS_OUT_PIN);
	DL_GPIO_initDigitalInputFeatures(GS_OUT_IOMUX,
		DL_GPIO_INVERSION_DISABLE,
		DL_GPIO_RESISTOR_NONE,
		DL_GPIO_HYSTERESIS_DISABLE,
		DL_GPIO_WAKEUP_DISABLE);

	/* Clear all address lines initially (select channel 0) */
	DL_GPIO_clearPins(GS_PORT, GS_AD0_PIN | GS_AD1_PIN | GS_AD2_PIN);
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
