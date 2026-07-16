#ifndef _GRAYSCALE_H
#define _GRAYSCALE_H

#include "ti_msp_dl_config.h"

#define GRAYSCALE_SENSOR_CHANNELS 8

/*
 * Correct pin mapping (all on GPIOA):
 *   AD0 = PA9  (IOMUX_PINCM20)  — mux channel-select bit 0
 *   AD1 = PA8  (IOMUX_PINCM19)  — mux channel-select bit 1
 *   AD2 = PA12 (IOMUX_PINCM34)  — mux channel-select bit 2
 *   OUT = PA27 (IOMUX_PINCM60)  — sensor output (analog mux common)
 */
#define GS_PORT               GPIOA

#define GS_AD0_PIN            DL_GPIO_PIN_9
#define GS_AD0_IOMUX          (IOMUX_PINCM20)

#define GS_AD1_PIN            DL_GPIO_PIN_8
#define GS_AD1_IOMUX          (IOMUX_PINCM19)

#define GS_AD2_PIN            DL_GPIO_PIN_12
#define GS_AD2_IOMUX          (IOMUX_PINCM34)

#define GS_OUT_PIN            DL_GPIO_PIN_27
#define GS_OUT_IOMUX          (IOMUX_PINCM60)

void Grayscale_Sensor_Init(void);
void Grayscale_Sensor_Read_All(uint16_t *sensor_values);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);

#endif /* _GRAYSCALE_H */
