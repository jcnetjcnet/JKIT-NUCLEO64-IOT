/*
 * led_fn.c
 *
 *  Created on: 2022. 1. 31.
 *      Author: isjeon
 */


#include "main.h"
#include "uart.h"
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
static uint8_t tmp1[128],tmp4[128];
void rs485_fn(int ac, char *av[])
{
    uint32_t led;
    int i;
    printf("RS485..\n");
    if(ac == 3 && !strcmp("tx",av[1]))
    {
    	if(av[2][0] == '1') // Uart1 tx
		{
			printf("Tx uart1\n");
			sprintf(tmp1,"MSG=[Uart 1 to Uart 4]\n");
            UART1_TXEN_GPIO_Port->BSRR =  UART1_TXEN_Pin;
            HAL_UART_Transmit(&huart1,(uint8_t *)tmp1,strlen(tmp1),1000);
            HAL_Delay(1);
            UART1_TXEN_GPIO_Port->BSRR =  UART1_TXEN_Pin << 16;
		}
		else // Uart4 tx
		{
			printf("Tx uart4\n");
			sprintf(tmp4,"MSG=[Uart 4 to Uart 1]\n");
            UART4_TXEN_GPIO_Port->BSRR =  UART4_TXEN_Pin;
            HAL_UART_Transmit(&huart4,(uint8_t *)tmp4,strlen(tmp4),1000);
            HAL_Delay(1);
            UART4_TXEN_GPIO_Port->BSRR =  UART4_TXEN_Pin << 16;
		}
    }
    else
    {
    	printf("Usage : rs485 tx [1/3]\n");
    }
}

