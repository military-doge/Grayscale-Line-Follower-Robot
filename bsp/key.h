#ifndef _KEY_H
#define _KEY_H
#include "ti_msp_dl_config.h"
#include "board.h"
#define KEY  DL_GPIO_readPins(KEY_PORT,KEY_key_PIN)
u8 click_N_Double (u8 time);
u8 click(void);
u8 Long_Press(void);
void Key(void);
#endif
