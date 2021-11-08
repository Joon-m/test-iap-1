#ifndef __UART1_H
#define __UART1_H

#include "common.h"

extern uint8_t uart_rec_buf[20];
extern uint8_t uart_rec_num;
extern uint8_t UART_DONE;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
