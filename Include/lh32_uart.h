#ifndef _LH32_UART_H
#define _LH32_UART_H

#include "lh32m0xx_lhl.h"

#define MAX_COMM_SIZE 256                   /* 通信环形缓冲区长度 */

extern uint8_t commBuffer[MAX_COMM_SIZE];

void uart_init(void);


#endif
