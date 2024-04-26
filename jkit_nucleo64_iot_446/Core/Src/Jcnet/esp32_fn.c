/*
 * led_fn.c
 *
 *  Created on: 2022. 1. 31.
 *      Author: isjeon
 */


#include "main.h"
#include "uart.h"
#include "jcnet.h"

#define AT_UART_RX_Q esp32_uart3
extern UART_HandleTypeDef huart3;
extern uart_rx_queue_t esp32_uart3;

extern void ESP32_send_string(char *str);
extern char *ESP32_getVersion();
extern char *ESP32_apList();
uint8_t * ESP32_C_getVersion();
char *ESP32_C_getAPList();
extern int tcpserver_init();
extern void tcpclient_init();
extern void udpclient_init();
extern void httpget_init();

extern int is_available(uart_rx_queue_t *Q);
extern int delete_uart_Q(uart_rx_queue_t *Q);
extern uart_rx_queue_t stdin_uart;

void wait_input()
{
	while(1)
	{
		if(is_available(&stdin_uart)) {
		  while(is_available(&stdin_uart))
		  {
			  int ch;
			  ch = delete_uart_Q(&stdin_uart);
		  }
          break;
		}
	}
}
void esp32_atlib_fn(int ac, char *av[])
{
	// DTR : PA15
	// RTS : PB7
	// DTR RTS  EN  IO0
	//  1   1    1   1
	//  0   0    1   1
	//  1   0    0   1
	//  0   1    1   0 (IO0가 1인 상태에서 reset 풀리게
	if(!strcmp(av[1],"dtr"))
	{
		printf("DTR RTS  EN IO0\n");
		printf(" 1   1   1   1\n");
		GPIOA->BSRR = (1 << 15) << 0 ;	 GPIOB->BSRR = (1 << 7) << 0 ; wait_input();

		printf(" 0   0   1   1\n");
		GPIOA->BSRR = (1 << 15) << 16;	 GPIOB->BSRR = (1 << 7) << 16; wait_input();

		printf(" 1   0   0   1\n");
		GPIOA->BSRR = (1 << 15) << 0;	 GPIOB->BSRR = (1 << 7) << 16; wait_input();

		printf(" 0   1   1   0\n");
		GPIOA->BSRR = (1 << 15) << 16;	 GPIOB->BSRR = (1 << 7) << 0; wait_input();
	}
	if(!strcmp(av[1],"hreset"))
	{

	 GPIOA->BSRR = (1 << 15);
	 GPIOB->BSRR = (1 << 7) << 16;
	 HAL_Delay(10);
	 GPIOA->BSRR = (1 << 15) << 16;
	 HAL_Delay(1000);
//	 esp32_uart_reinit();

	}
#ifdef ARDUINO_CPP_ATLIB_USE
	if(!strcmp(av[1],"ver"))
	{
		char *res = ESP32_getVersion();
		printf("Version=[%s]\n",res);
	}
	if(!strcmp(av[1],"ap"))
	{
		char *res = ESP32_apList();
		printf("AP=[%s]\n",res);
	}
#endif
	if(!strcmp(av[1],"cver"))
	{
		char *res = ESP32_C_getVersion();
		printf("Version=[%s]\n",res);
	}

	if(!strcmp(av[1],"cap"))
	{
		char *res = ESP32_C_getAPList();
		printf("AP=[%s]\n",res);
	}
	if(!strcmp(av[1],"tcpserver"))
	{
		tcpserver_init();
	}
	if(!strcmp(av[1],"tcpclient"))
	{
		tcpclient_init();
	}
	if(!strcmp(av[1],"udpclient"))
	{
		udpclient_init();
	}
	if(!strcmp(av[1],"http"))
	{
		httpget_init();
	}
}
void esp32_fn(int ac, char *av[])
{
    uint32_t tick;
    int i;
    char tmp1[100];
    if(ac > 1)
    {
    	strcpy(tmp1,av[1]);
    	for( i = 0 ; i < ac - 2; i ++) sprintf(tmp1+strlen(tmp1)," %s",av[i+2]);
    	sprintf(tmp1+strlen(tmp1),"\r\n");
//        printf("[ESP32]=[%s]",tmp1);

        HAL_UART_Transmit(&huart3,(uint8_t *)tmp1,strlen(tmp1),1000);

        tick = 0;
        while(tick < 1000)
        {
            if(is_available(&AT_UART_RX_Q))
            {
                int ch;
                ch = delete_uart_Q(&AT_UART_RX_Q);
                printf("%c",ch);
                tick = 0;
            }
            else
            {
            	tick ++;
            	HAL_Delay(1);
            }
        }
    }
}

