/*
 * tcpserver.c
 *
 *  Created on: Apr 19, 2024
 *      Author: isjeon
 */
#include "main.h"

#include <stdbool.h>

#define  DEFAULT_PATTERN  3
#define SSID "jcnet"
#define PASSWORD "jcnet100"

extern bool ESP8266_setOprToSoftAP(uint8_t pattern1, uint8_t pattern2);
extern bool ESP8266_setOprToStationSoftAP(uint8_t pattern1, uint8_t pattern2);
extern bool ESP8266_joinAP(uint8_t *ssid, uint8_t *pwd, uint8_t pattern);
extern uint8_t *ESP8266_getLocalIP();
extern bool ESP8266_enableMUX();
extern bool ESP8266_startTCPServer(uint32_t port);
int tcpserver_loop();
int tcpserver_init()
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
		return -1;
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
	if(ESP8266_startTCPServer(8090))
	{
	    printf("Start TCP server OK\n");
	}
	else
	{
		printf("start TCP server failed\n");
		return -1;
	}
	if (ESP8266_setTCPServerTimeout(10))
	{
	    printf("Set TCP server timeout 10 seconds\n");
	}
	else
	{
		printf("Set TCP server timeout failed\n");
	}
	tcpserver_loop();
}
#include "uart.h"
extern int is_available(uart_rx_queue_t *Q);
extern uart_rx_queue_t stdin_uart;

extern bool ESP8266_releaseTCP_mux(uint8_t mux_id);
extern uint32_t ESP8266_recv_mux_p(uint8_t *coming_mux_id, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout);
extern bool ESP8266_send_mux(uint8_t mux_id, const uint8_t *buffer, uint32_t len);
uint8_t *ESP8266_getIPStatus();
void loop()
{
  uint8_t buffer[128] = {0};
  uint8_t mux_id;

  uint32_t len = ESP8266_recv_mux_p(&mux_id, buffer, sizeof(buffer), 100);

  if (len > 0)
  {
    printf("Status:[");
    printf("%s",ESP8266_getIPStatus());
    printf("]\n");

    printf("Received from :");
    printf("%d\n",mux_id);
    printf("[");

    for (uint32_t i = 0; i < len; i++)
    {
      printf("%c",(char)buffer[i]);
    }

    printf("]\n");

    if (ESP8266_send_mux(mux_id, buffer, len))
    {
    	printf("Send back OK\n");
    }
    else
    {
    	printf("Send back failed\n");
    }

    if (ESP8266_releaseTCP_mux(mux_id))
    {
      printf("Release TCP ");
      printf("%d",mux_id);
      printf(" OK\n");
    }
    else
    {
      printf("Release TCP ");
      printf("%d",mux_id);
      printf(" failed\n");
    }

    printf("Status:[");
    printf("%s",ESP8266_getIPStatus());
    printf("]\n");
  }
}
int tcpserver_loop()
{
	while(1)
	{
		if(is_available(&stdin_uart)) break;
		loop();

	}
}

