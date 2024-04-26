/*
 * esp_at_if.cpp
 *
 *  Created on: Apr 16, 2024
 *      Author: isjeon
 */

#include "WString.h"
extern "C" {
#include "main.h"
#include "uart.h"
#define AT_UART_CH huart3
#define AT_UART_RX_Q esp32_uart3

//	extern UART_HandleTypeDef huart4;
	extern UART_HandleTypeDef huart2;
	extern UART_HandleTypeDef huart3;
	extern uart_rx_queue_t AT_UART_RX_Q;
	extern int is_available(uart_rx_queue_t *Q);
	extern int delete_uart_Q(uart_rx_queue_t *Q);
	char *ESP32_getVersion();
	char *ESP32_apList();
	char ESP32_version[256];
	char ESP32_ap_list[3*1024];
}

class Serial {
private:
	int m_idx;
	UART_HandleTypeDef *m_handle;
public:
	Serial(int idx)
	{
		m_idx = idx;
	}
	Serial(UART_HandleTypeDef *p_Handle)
	{
		m_handle = p_Handle;
	}
	Serial()
	{
		m_handle = &AT_UART_CH;
	}
	int available();
	void print(String str);
	void print(uint32_t v);
	void println(String str);
	void println(uint32_t v);
	char read();
	void write(char ch);
};
Serial myDebug(&huart2);
//Serial myTest(&huart3);
Serial myAt(&AT_UART_CH);
//#include "WString.h"
#include "ESP_AT_Lib.h"



extern "C"
void ESP32_send_string(char *str)
{
	String s;
	s = "";
	while(*str) s.concat(*str++);
	myAt.print(s);
}

String ESP8266::getVersion();
ESP8266 myESP8266;
char *ESP32_getVersion()
{
	String sVersion;
	sVersion = myESP8266.getVersion();
	sVersion.toCharArray(ESP32_version,sizeof(ESP32_version));
	myDebug.println(sVersion);
	return ESP32_version;
}
char *ESP32_apList()
{
	String sApList;
	sApList = myESP8266.getAPList();
	sApList.toCharArray(ESP32_ap_list,sizeof(ESP32_ap_list));
	myDebug.println(sApList);
	return ESP32_ap_list;
}
ESP8266::ESP8266()
{
	ESP8266::m_puart = &myAt;
}

int Serial::available()
{
	return is_available(&AT_UART_RX_Q);
//	return 0;
}

void Serial::print(String str)
{
	char buf[1024];
	str.toCharArray(buf,1024);

	HAL_UART_Transmit(m_handle,(uint8_t *)buf,strlen(buf),1000);

}
void Serial::print(uint32_t v)
{
	char buf[1024];
	sprintf(buf,"%d",v);
//    HAL_UART_Transmit(&AT_UART_CH,(uint8_t *)buf,strlen(buf),1000);
    HAL_UART_Transmit(m_handle,(uint8_t *)buf,strlen(buf),1000);
}

void Serial::println(String str)
{
	char buf[1024];
	str.toCharArray(buf,1024);
	strcat(buf,"\r\n");
    HAL_UART_Transmit(m_handle,(uint8_t *)buf,strlen(buf),1000);
}
void Serial::println(uint32_t v)
{
	char buf[1024];
	sprintf(buf,"%d\r\n",v);
    HAL_UART_Transmit(m_handle,(uint8_t *)buf,strlen(buf),1000);
}

char Serial::read()
{
	char ch;
	if(m_handle == &AT_UART_CH)
		ch = delete_uart_Q(&AT_UART_RX_Q);
	return ch;
}

void Serial::write(char ch)
{
	char buf[1024];
	sprintf(buf,"%c",ch);
    HAL_UART_Transmit(m_handle,(uint8_t *)buf,strlen(buf),1000);
}

