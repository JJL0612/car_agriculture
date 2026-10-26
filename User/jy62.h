#ifndef __JY62_H
#define __JY62_H

#include "usart.h"

extern uint8_t uart_rx_buf[256];

extern float yaw;

void jy62_init(void);
void cyz_process_data(uint8_t *data, uint16_t size);
void yaw_Calculate(void);

#endif /* __JY62_H */