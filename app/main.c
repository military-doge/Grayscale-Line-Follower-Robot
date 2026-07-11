#include "ti_msp_dl_config.h"
#include "board.h"
#include "oled.h"
#include "led.h"
#include <string.h>

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();

    uint32_t counter = 0;

    while (1) {
        memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
        OLED_ShowString(0, 0, (const uint8_t *)"HELLO WHEELTEC");
        OLED_ShowNumber(30, 20, counter, 8, 12);
        OLED_Refresh_Gram();
        counter++;
        LED_Flash(100);
        delay_ms(10);
    }
}

