#ifndef BSP_USART_H__
#define BSP_USART_H__

#include "main.h"
#include "usart.h"
#include "dma.h"

#define UART_RX_DMA_SIZE (1024)

#define SBUS_HEAD 0X0F
#define SBUS_END 	0X00

#define USART5_BUFLEN 18

//开启遥控器不定长接收
void remoter_start(void);

#endif
