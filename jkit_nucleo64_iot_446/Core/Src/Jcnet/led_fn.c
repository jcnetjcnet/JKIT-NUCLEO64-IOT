/*
 * led_fn.c
 *
 *  Created on: 2022. 1. 31.
 *      Author: isjeon
 */


#include "main.h"
#include "uart.h"
extern int is_available(uart_rx_queue_t *Q);
extern uart_rx_queue_t stdin_uart;
void led_fn(int ac, char *av[])
{
    uint32_t led;
    int i;
    printf("LED..\n");
    led = 1;
    if(ac == 2)
    {
    	sscanf(av[1],"%d",&led);
    	while(1)
    	{
    	GPIOB->BSRR = 1 << led;
    	HAL_Delay(200);
    	GPIOB->BSRR = (1 << led) << 16;
    	HAL_Delay(200);
        if(is_available(&stdin_uart)) break;
    	}
    }
    for( i = 0 ; i < 4 ; i ++, led <<= 1)
    {
            GPIOB->ODR = (led & 0xf);
            HAL_Delay(200);
    }
    GPIOB->ODR = 0;
}

void switch_fn(int ac, char *av[])
{
        uint32_t prev,flag = 0;
        prev = HAL_GetTick();
        while(1)
        {
        	if((INPUT1_GPIO_Port->IDR & INPUT1_Pin) == 0) LED1_GPIO_Port-> BSRR = LED1_Pin;
        	else LED1_GPIO_Port-> BSRR = LED1_Pin << 16;
        	if((INPUT2_GPIO_Port->IDR & INPUT2_Pin) == 0) LED2_GPIO_Port-> BSRR = LED2_Pin;
        	else  LED2_GPIO_Port-> BSRR = LED2_Pin << 16;

        }
}
