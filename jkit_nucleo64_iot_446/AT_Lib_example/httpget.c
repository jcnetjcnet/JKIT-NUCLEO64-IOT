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
#define HOST_NAME "www.yahoo.com"
#define HOST_PORT (80)

bool ESP8266_createTCP(uint8_t * addr, uint32_t port);
extern bool ESP8266_setOprToStationSoftAP(uint8_t pattern1, uint8_t pattern2);
extern bool ESP8266_joinAP(uint8_t *ssid, uint8_t *pwd, uint8_t pattern);

void httpget_init()
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

	httpclient_loop();

}
#include "uart.h"
extern int is_available(uart_rx_queue_t *Q);
extern uart_rx_queue_t stdin_uart;
extern bool ESP8266_createTCP_mux(uint8_t mux_id, uint8_t * addr, uint32_t port);
extern bool ESP8266_send_mux(uint8_t mux_id, const uint8_t *buffer, uint32_t len);
extern bool ESP8266_releaseTCP_mux(uint8_t mux_id);
void http_action()
{
	uint8_t buffer[128] = {0};
	uint8_t mux_id = 0;

	printf("Create TCP ");
	if(ESP8266_createTCP_mux(mux_id, HOST_NAME, HOST_PORT ))
	{
		printf("OK\n");
	}
	else
	{
		printf("failed\n");
	}
	char hello[] = "GET / HTTP/1.1\r\nHost: www.yahoo.com\r\nConnection: close\r\n\r\n";
    if (ESP8266_send_mux(mux_id, hello, strlen(hello)))
    {
    	printf("Send OK\n");
    }
    else
    {
    	printf("Send  failed\n");
    }

    uint32_t len = ESP8266_recv_mux_p(&mux_id, buffer, sizeof(buffer), 10000);

    if (len > 0)
    {

    	printf("Received:[");

    	for (uint32_t i = 0; i < len; i++)
    	{
    		printf("%c",(char)buffer[i]);
    	}

    	printf("]\n");
    }
    printf("Release TCP ");
    if (ESP8266_releaseTCP_mux(mux_id))
    {
    	printf(" OK\n");
    }
    else
    {
    	printf(" failed\n");
    }

    HAL_Delay(5000);
}
void httpclient_loop()
{
	while(1)
	{
		if(is_available(&stdin_uart)) break;
		http_action();

	}
}
