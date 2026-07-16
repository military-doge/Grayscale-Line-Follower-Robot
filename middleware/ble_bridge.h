#ifndef _BLE_BRIDGE_H_
#define _BLE_BRIDGE_H_

#include "board.h"

void ble_bridge_init(void);
void ble_bridge_poll(void);

/* 供外部模块注册编码器读数 */
int32_t ble_bridge_get_encoder_a(void);
int32_t ble_bridge_get_encoder_b(void);

#endif /* _BLE_BRIDGE_H_ */
