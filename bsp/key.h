#ifndef _KEY_H
#define _KEY_H
#include "ti_msp_dl_config.h"
#include "board.h"

#define KEY  DL_GPIO_readPins(KEY_PORT,KEY_key_PIN)

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_CLICK = 1,
    KEY_EVENT_DOUBLE = 2,
    KEY_EVENT_LONG   = 3
} KeyEvent;

u8 click_N_Double (u8 time);
u8 click(void);
u8 Long_Press(void);
KeyEvent Key(void);
#endif
