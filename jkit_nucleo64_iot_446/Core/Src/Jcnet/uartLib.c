/*
 * uartLib.c
 *
 *  Created on: Jan 25, 2022
 *      Author: isjeon
 */


#include "main.h"
#include "uart.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
__IO uint8_t rxBuff1[2];
__IO uint8_t rxBuff2[2];
__IO uint8_t rxBuff3[2];
__IO uint8_t rxBuff4[2];
__IO uint32_t rx_flag1, rx_char1;

uint8_t stdin_q_buf[64];
uart_rx_queue_t stdin_uart =
{
		.data = stdin_q_buf,
		.size = 64
};
uint8_t rs485_1_q_buf[64];
uart_rx_queue_t rs485_uart1 =
{
		.data = rs485_1_q_buf,
		.size = 64
};
uint8_t rs485_4_q_buf[64];
uart_rx_queue_t rs485_uart4 =
{
		.data = rs485_4_q_buf,
		.size = 64
};
#define ESP32_UART_RX_Q_SZ (1024*3)
uint8_t esp32_q_buf[ESP32_UART_RX_Q_SZ];
uart_rx_queue_t esp32_uart3 =
{
		.data = esp32_q_buf,
		.size = ESP32_UART_RX_Q_SZ
};

int insert_uart_Q(uart_rx_queue_t *Q, uint8_t ch)
{
        if((Q->wr + 1) % Q->size == Q->rd)
        {
                return -1; // Full
        }
        Q->data[Q->wr] = ch;
        Q->wr = (Q->wr + 1) % Q->size;
        return 0;
}

int insert_uart_AT_Q(uint8_t ch)
{
	return insert_uart_Q(&esp32_uart3, ch);
}

int delete_uart_Q(uart_rx_queue_t *Q)
{
        int ch;
        if(Q->wr == Q->rd) return -1;
        ch = Q->data[Q->rd];
        Q->rd = (Q->rd + 1) % Q->size;
        return ch;
}

int is_available(uart_rx_queue_t *Q)
{
        return (Q->wr != Q->rd);
}

void uart_rx_ready_IT(UART_HandleTypeDef *huart,uint8_t *buf)
{
         HAL_UART_Receive_IT(huart, buf , 1);
}

#include <stdio.h>
int _write(int file, char *data, int len)
{
    int bytes_written;

    HAL_UART_Transmit(&huart2,(uint8_t *)data, len,1000);

    bytes_written = len;
    return bytes_written;
}

void my_putchar(char c)
{
        _write(1,&c,1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
        if(huart == &huart1)
        {
            rx_flag1 = 1;
            insert_uart_Q(&rs485_uart1,rxBuff1[0]);
            HAL_UART_Receive_IT(&huart1, rxBuff1 , 1);
        }
        if(huart == &huart2)
        {
                insert_uart_Q(&stdin_uart,rxBuff2[0]);
                HAL_UART_Receive_IT(&huart2, rxBuff2 , 1);
        }
#if 0
        if(huart == &huart3)
        {
        	volatile unsigned int i;
            insert_uart_Q(&rs485_uart3,rxBuff3[0]);
            HAL_UART_Receive_IT(&huart3, rxBuff3 , 1);
        }
#endif
        if(huart == &huart4)
        {
        	volatile unsigned int i;
            insert_uart_Q(&rs485_uart4,rxBuff4[0]);
            HAL_UART_Receive_IT(&huart4, rxBuff4 , 1);
        }
}
esp32_uart_reinit()
{
	HAL_UART_MspDeInit(&huart3);
	JCNET_USART3_UART_Init();
	uart_rx_ready_IT(&huart3,rxBuff3);
}
