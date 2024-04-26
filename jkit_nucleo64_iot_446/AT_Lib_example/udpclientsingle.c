/*
 * tcpclientsingle.c
 *
 *  Created on: Apr 19, 2024
 *      Author: isjeon
 */


#include "main.h"

#include <stdbool.h>
#define  DEFAULT_PATTERN  3
#define SSID "jcnet"
#define PASSWORD "jcnet100"
#define HOST_NAME "192.168.0.29"
#define HOST_PORT (8080)

extern bool ESP8266_setOprToStationSoftAP(uint8_t pattern1, uint8_t pattern2);
extern bool ESP8266_joinAP(uint8_t *ssid, uint8_t *pwd, uint8_t pattern);
extern void udpclient_loop();
void udpclient_init()
{
	if(ESP8266_setOprToStationSoftAP(DEFAULT_PATTERN,DEFAULT_PATTERN))
	{
		printf("Set AP/STA Mode OK\n");
	}
	else {
		printf("Set AP/STA Mode failed\n");
		return -1;
	}
	if(ESP8266_joinAP(SSID,PASSWORD, DEFAULT_PATTERN ))
	{
		printf("Connect to WiFi ok\n");
		printf("IP addr = %s\n",ESP8266_getLocalIP());
	}
	else
	{
		printf("Connect to WiFi failed \n");
	}
	if(ESP8266_enableMUX())
	{
		printf("enableMux OK\n");
	}
	else
	{
		printf("enableMux failed\n");
		return -1;
	}

	udpclient_loop();

}
#include "uart.h"
extern int is_available(uart_rx_queue_t *Q);
extern uart_rx_queue_t stdin_uart;

extern bool ESP8266_registerUDP_mux(uint8_t mux_id, uint8_t * addr, uint32_t port);
extern bool ESP8266_unregisterUDP_mux(uint8_t mux_id);
extern bool ESP8266_send_mux(uint8_t mux_id, const uint8_t *buffer, uint32_t len);
extern uint32_t ESP8266_recv_mux(uint8_t mux_id, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout);

void udp_client()
{
	uint8_t buffer[128] = {0};
	uint8_t mux_id = 0;

	printf("Register UDP ");
	if(ESP8266_registerUDP_mux(mux_id, HOST_NAME, HOST_PORT ))
	{
		printf("OK\n");
	}
	else
	{
		printf("failed\n");
	}
	char hello[] = "Hello, this is jcnet udp client !";
    if (ESP8266_send_mux(mux_id, hello, strlen(hello)))
    {
    	printf("Send OK\n");
    }
    else
    {
    	printf("Send  failed\n");
    }

    uint32_t len = ESP8266_recv_mux(mux_id, buffer, sizeof(buffer), 10000);

    if (len > 0)
    {

    	printf("Received:[");

    	for (uint32_t i = 0; i < len; i++)
    	{
    		printf("%c",(char)buffer[i]);
    	}

    	printf("]\n");
    }
    printf("Unregister UDP ");
    if (ESP8266_unregisterUDP_mux(mux_id))
    {
    	printf(" OK\n");
    }
    else
    {
    	printf(" failed\n");
    }

    HAL_Delay(5000);
}
void udpclient_loop()
{
	while(1)
	{
		if(is_available(&stdin_uart)) break;
		udp_client();

	}
}
