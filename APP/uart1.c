#include "uart1.h"

static uint8_t uart_rec_buf[20];
static uint8_t uart_rec_num=0;
static uint8_t UART_DONE=0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART1)	// 判断是由哪个串口触发的中断
	{
		HAL_UART_Transmit(&huart1,aRxBuffer1,1,100);	// 接收到数据马上使用串口1发送出去
		HAL_UART_Receive_IT(&huart1,aRxBuffer1,1);		// 重新使能串口1接收中断
	}
}

