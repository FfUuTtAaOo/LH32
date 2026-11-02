#ifndef _LH32_UART_H
#define _LH32_UART_H

#include "lh32m0xx_lhl.h"

#define UART_SIZE 256                   /* RX 环形缓冲区长度 */

extern uint8_t uart_buffer[UART_SIZE];
extern uint8_t uart_rx_count;
extern uint8_t uart_tx_count;

void uart_init(void);

#endif
