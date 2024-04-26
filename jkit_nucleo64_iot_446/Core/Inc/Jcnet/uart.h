/*
 * uart.h
 *
 *  Created on: Jan 25, 2022
 *      Author: isjeon
 */

#ifndef INC_JCNET_UART_H_
#define INC_JCNET_UART_H_


//#define UART_Q_SZ 4096
typedef struct _uart_tag_ {
        uint8_t *data; // [UART_Q_SZ];
        uint32_t size;
        uint32_t wr, rd;
} uart_rx_queue_t;

#endif /* INC_JCNET_UART_H_ */
