/****************************************************************************************************************************
  ESP_AT_Lib_Impl.h - Dead simple ESP8266/ESP32-AT-command wrapper
  For WizFi360/ESP8266/ESP32-AT-command running shields

  ESP_AT_Lib is a wrapper library for the WizFi360/ESP8266/ESP32 AT-command shields

  Based on and modified from ESP8266 https://github.com/esp8266/Arduino/releases
  Built by Khoi Hoang https://github.com/khoih-prog/ESP_AT_Lib
  Licensed under MIT license

  @file ESP_AT_Lib.cpp
  @brief The implementation of class ESP_AT for ESP8266/ESP32-AT-command.
  @author Wu Pengfei<pengfei.wu@itead.cc>
  @date 2015.02

  @par Copyright:
  Copyright (c) 2015 ITEAD Intelligent Systems Co., Ltd. \n\n
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation; either version 2 of
  the License, or (at your option) any later version. \n\n
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.

  Version: 1.5.1

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K Hoang      12/02/2020 Initial coding for ESP8266/ESP32-AT shields to support Mega, nRF52, SAMD, DUE, STM32, etc.
  1.1.0   K Hoang      10/05/2021 Add support to BOARD_SIPEED_MAIX_DUINO and RASPBERRY_PI_PICO
  1.2.0   K Hoang      17/05/2021 Add support to RP2040-based boards using Arduino-mbed RP2040 core. Fix compiler warnings
  1.3.0   K Hoang      29/05/2021 Add support to RP2040-based Nano_RP2040_Connect using Arduino-mbed RP2040 core
  1.4.0   K Hoang      13/08/2021 Add support to Adafruit nRF52 core v0.22.0+
  1.4.1   K Hoang      10/10/2021 Update `platform.ini` and `library.json`
  1.5.0   K Hoang      19/01/2023 Add support to WizNet WizFi360 such as WIZNET_WIZFI360_EVB_PICO
  1.5.1   K Hoang      19/01/2023 Fix mistakes
 *****************************************************************************************************************************/

#ifndef __ESP_AT_LIB_IMPL_H__
#define __ESP_AT_LIB_IMPL_H__

////////////////////////////////////////
#include "main.h"
#include <stdbool.h>
#include <stdlib.h>

#define USE_ESP32_AT 1

typedef struct at_uart_tag {
	void (*println)(char *);
	void (*print)(char *);
	void (*println_int)(int);
	void (*print_int)(int);
	int (*available)();
	int (*read)();
	void (*write)(uint8_t *ch, int len);
} at_uart;
at_uart *m_puart, m_uart;


static uint8_t ssid[128];
static uint8_t dhcp[128];
static uint8_t mac[16];
static uint8_t ip[16];
#if 0
static uint8_t version[256];
static uint8_t list[3*1024];
#else
char ESP32_version[256];
char ESP32_ap_list[3*1024];
#endif

#define F(x) (x)
bool eAT();
bool eATRST();
bool eATGMR(char *version);
bool eATGSLP(uint32_t time);
bool eATE(uint8_t mode);
bool eATRESTORE();
bool eATSETUART(uint32_t baudrate, uint8_t pattern);
bool qATCWMODE(uint8_t *mode, uint8_t pattern);
bool eATCWMODE(uint8_t *list);
bool sATCWMODE(uint8_t mode, uint8_t pattern);
bool qATCWJAP(uint8_t *ssid, uint8_t pattern);
bool sATCWJAP(uint8_t *ssid, uint8_t *pwd, uint8_t pattern);
bool eATCWLAP(uint8_t *list);
bool eATCWQAP();
bool qATCWSAP(uint8_t *List, uint8_t pattern);
bool sATCWSAP(uint8_t *ssid, uint8_t *pwd, uint8_t chl, uint8_t ecn, uint8_t pattern);
bool eATCWAUTOCONN(uint8_t en);
bool eATCWLIF(uint8_t *list);
bool qATCWDHCP(uint8_t *list, uint8_t pattern);
bool sATCWDHCP(uint8_t op, uint8_t mode, uint8_t pattern);
bool qATCIPSTAMAC(uint8_t *mac, uint8_t pattern);
bool eATCIPSTAMAC(uint8_t * mac, uint8_t pattern);
bool qATCIPSTAIP(uint8_t *ip, uint8_t pattern);
bool eATCIPSTAIP(uint8_t * ip, uint8_t * gateway, uint8_t * netmask, uint8_t pattern);
bool qATCIPAP(uint8_t *ip, uint8_t pattern);
bool eATCIPAP(uint8_t * ip, uint8_t pattern);
bool eCWSTARTSMART(uint8_t type);
bool eCWSTOPSMART();
bool eATCIPSTATUS(uint8_t *list);
bool sATCIPSTARTSingle(uint8_t * type, uint8_t * addr, uint32_t port);
bool sATCIPSENDSingle(const uint8_t *buffer, uint32_t len);
bool sATCIPSENDMultiple(uint8_t mux_id, const uint8_t *buffer, uint32_t len);
bool sATCIPSENDSingleFromFlash(const uint8_t *buffer, uint32_t len);
bool sATCIPSENDMultipleFromFlash(uint8_t mux_id, const uint8_t *buffer, uint32_t len);
bool sATCIPCLOSEMultiple(uint8_t mux_id);
bool eATCIPCLOSESingle();
bool sATCIPSTARTMultiple(uint8_t mux_id, uint8_t * type, uint8_t * addr, uint32_t port);
bool eATCIFSR(uint8_t *list);
bool sATCIPMUX(uint8_t mode);
bool sATCIPSERVER(uint8_t mode, uint32_t port);
bool sATCIPMODE(uint8_t mode);
bool eATSAVETRANSLINK(uint8_t mode, uint8_t * ip, uint32_t port);
bool sATCIPSTO(uint32_t timeout);
uint32_t ESP8266_recv(uint8_t *buffer, uint32_t buffer_size, uint32_t timeout);
uint32_t ESP8266_recv_mux(uint8_t mux_id, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout);
uint32_t ESP8266_recv_mux_p(uint8_t *coming_mux_id, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout);
bool eATSAVETRANSLINK(uint8_t mode, uint8_t * ip, uint32_t port);
bool eATPING(uint8_t * ip);
bool sATCIPSTO(uint32_t timeout);

#include "uart.h"


#define AT_LIB_LOGDEBUG(x)
#define F(x) (x)
#define AT_UART_CH huart3
#define AT_UART_RX_Q esp32_uart3
extern uart_rx_queue_t AT_UART_RX_Q;

static uint32_t checkIPD(uint8_t *data);

UART_HandleTypeDef huart3;


static uint32_t millis()
{
	return HAL_GetTick();
}


static void delay(uint32_t x)
{
	HAL_Delay(x);
}
#include <string.h>
static int indexOfString(char *src, char *target)
{
	int i = 0;
	int num;
	num = strlen(src) - strlen(target);
	for( i = 0 ; i <= num ; i ++)
	{
		if(!strncmp(src+i,target,strlen(target))) return i;
	}
	return -1;
}

static int indexOfChar(char *src, char t,int start)
{
	int i;
	for( i = start ; i < strlen(src) ; i ++)
	{
		if(src[i] == t) return i;
	}
	return -1;
}

void send_string_with_nl(char *s)
{
	char buf[1024];
	sprintf(buf,"%s\r\n",s);
	HAL_UART_Transmit(&AT_UART_CH,(uint8_t *)buf,strlen(buf),1000);
}
void send_string(char *s)
{
	HAL_UART_Transmit(&AT_UART_CH,(uint8_t *)s,strlen(s),1000);
}

void send_int_with_nl(int v)
{
	char buf[1024];
	sprintf(buf,"%d\r\n",v);
	HAL_UART_Transmit(&AT_UART_CH,(uint8_t *)buf,strlen(buf),1000);
}
void send_int(int v)
{
	char buf[1024];
	sprintf(buf,"%d",v);
	HAL_UART_Transmit(&AT_UART_CH,(uint8_t *)buf,strlen(buf),1000);
}

char AT_ch_read()
{
	char ch;
	ch = delete_uart_Q(&AT_UART_RX_Q);
	return ch;
}
void AT_ch_write(char *p, int len)
{
	HAL_UART_Transmit(&AT_UART_CH,(uint8_t *)p,len,1000);
}
extern int is_available(uart_rx_queue_t *Q);
int AT_ch_available()
{
	return is_available(&AT_UART_RX_Q);
}

void ESP8266_ESP8266()
{
//  m_onData = NULL;
//  m_onDataPtr = NULL;
	m_uart.println = send_string_with_nl;
	m_uart.print   = send_string;
	m_uart.println_int = send_int_with_nl;
	m_uart.print_int   = send_int;
	m_uart.read    = AT_ch_read;
	m_uart.write    = AT_ch_write;
	m_uart.available = 	AT_ch_available;
	m_puart = &m_uart;
}

////////////////////////////////////////

bool ESP8266_kick()
{
  return eAT();
}

////////////////////////////////////////

bool ESP8266_restart()
{
  unsigned long start;
  if (eATRST())
  {
    delay(2000);
    start = millis();

    while (millis() - start < 3000)
    {
      if (eAT())
      {
        delay(1500); /* Waiting for stable */
        return true;
      }

      delay(100);
    }
  }
  
  return false;
}

////////////////////////////////////////

uint8_t * ESP32_C_getVersion()
{
//  String version;

  eATGMR(ESP32_version);
  
  return ESP32_version;
}

////////////////////////////////////////

bool ESP8266_setEcho(uint8_t mode)
{
  return eATE(mode);
}

////////////////////////////////////////

bool ESP8266_restore()
{
  return eATRESTORE();
}

////////////////////////////////////////

bool ESP8266_setUart(uint32_t baudrate, uint8_t pattern)
{
  return eATSETUART(baudrate, pattern);
}

////////////////////////////////////////

bool ESP8266_deepSleep(uint32_t time)
{
  return eATGSLP(time);
}

////////////////////////////////////////

bool ESP8266_setOprToStation(uint8_t pattern1, uint8_t pattern2)
{
  uint8_t mode;
  
  if (!qATCWMODE(&mode, pattern1))
  {
    return false;
  }

  if (mode == 1)
  {
    return true;
  }
  else
  {
    if (sATCWMODE(1, pattern2))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

////////////////////////////////////////

uint8_t *ESP8266_getWifiModeList()
{

  eATCWMODE(ESP32_ap_list);
  
  return ESP32_ap_list;
}

////////////////////////////////////////

bool ESP8266_setOprToSoftAP(uint8_t pattern1, uint8_t pattern2)
{
  uint8_t mode;

  if (!qATCWMODE(&mode, pattern1))
  {
    return false;
  }

  if (mode == 2)
  {
    return true;
  }
  else
  {
    if (sATCWMODE(2, pattern2) )
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

////////////////////////////////////////

bool ESP8266_setOprToStationSoftAP(uint8_t pattern1, uint8_t pattern2)
{
  uint8_t mode;

  if (!qATCWMODE(&mode, pattern1))
  {
    return false;
  }

  if (mode == 3)
  {
    return true;
  }
  else
  {
    if (sATCWMODE(3, pattern2) )
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

////////////////////////////////////////

uint8_t ESP8266_getOprMode(uint8_t pattern1)
{
  uint8_t mode;

  if (!qATCWMODE(&mode, pattern1))
  {
    return 0;
  }
  else
  {
    return mode;
  }
}

////////////////////////////////////////

uint8_t *ESP8266_getNowConecAp(uint8_t pattern)
{


  qATCWJAP(ssid, pattern);
  
  return ssid;
}

////////////////////////////////////////

char *ESP32_C_getAPList()
{

  eATCWLAP(ESP32_ap_list);
  
  return ESP32_ap_list;
}

////////////////////////////////////////

bool ESP8266_joinAP(uint8_t *ssid, uint8_t *pwd, uint8_t pattern)
{
  return sATCWJAP(ssid, pwd, pattern);
}

////////////////////////////////////////

bool ESP8266_leaveAP()
{
  return eATCWQAP();
}

////////////////////////////////////////

char  ESP8266_getSoftAPParam(uint8_t pattern)
{
  qATCWSAP(ESP32_ap_list, pattern);
  return ESP32_ap_list;
}

////////////////////////////////////////

bool ESP8266_setSoftAPParam(uint8_t *ssid, uint8_t *pwd, uint8_t chl, uint8_t ecn, uint8_t pattern)
{
  return sATCWSAP(ssid, pwd, chl, ecn, pattern);
}

////////////////////////////////////////

uint8_t *ESP8266_getJoinedDeviceIP()
{
  eATCWLIF(ESP32_ap_list);
  return ESP32_ap_list;
}

////////////////////////////////////////

uint8_t *ESP8266_getDHCP(uint8_t pattern)
{


  qATCWDHCP(dhcp, pattern);
  return dhcp;
}

////////////////////////////////////////

#if USE_ESP32_AT
// For ESP32-AT
/**
  Set the  state of DHCP.
  @param pattern -1, -2, -3 send "AT+CWDHCP=".
  @param op - 0 disable DHCP  - 1 enable DHCP.
  @param mode - Bit 0 : STA DHCP / Bit 1: SoftAP DHCP

  @retval true - success.
  @retval false - failure.
*/
bool ESP8266_setDHCP(uint8_t op, uint8_t mode, uint8_t pattern)
{
  return sATCWDHCP(op, mode, pattern);
}

#else
// For ESP8266-AT
/**
  Set the  state of DHCP.
  @param pattern -1 send "AT+CWDHCP_DEF=" -2 send "AT+CWDHCP_CUR=" -3 send "AT+CWDHCP=".
  @param mode - ‣ 0: Sets ESP8266 SoftAP, ‣ 1: Sets ESP8266 Station, ‣ 2: Sets both SoftAP and Station
  @param en - 0 disable DHCP  - 1 enable DHCP.
  @retval true - success.
  @retval false - failure.
*/
bool setDHCP(uint8_t mode, uint8_t en, uint8_t pattern)
{
  return sATCWDHCP(mode, en, pattern);
}
#endif

////////////////////////////////////////

bool ESP8266_setAutoConnect(uint8_t en)
{
  return eATCWAUTOCONN(en);
}

////////////////////////////////////////

uint8_t *ESP8266_getStationMac(uint8_t pattern)
{


  qATCIPSTAMAC(mac, pattern);
  
  return mac;
}

////////////////////////////////////////

bool ESP8266_setStationMac(uint8_t *mac, uint8_t pattern)
{
  return eATCIPSTAMAC(mac, pattern);
}

////////////////////////////////////////

uint8_t *ESP8266_getStationIp(uint8_t pattern)
{


  qATCIPSTAIP(ip, pattern);
  
  return ip;
}

////////////////////////////////////////

bool ESP826_setStationIp(uint8_t *ip, uint8_t *gateway, uint8_t *netmask, uint8_t pattern)
{
  return eATCIPSTAIP(ip, gateway, netmask, pattern);
}

////////////////////////////////////////

uint8_t * ESP8266_getAPIp(uint8_t pattern)
{
  qATCIPAP(ip, pattern);
  return ip;
}

////////////////////////////////////////

bool ESP8266_setAPIp(uint8_t * ip, uint8_t pattern)
{
  return eATCIPAP(ip, pattern);
}

////////////////////////////////////////

bool ESP8266_startSmartConfig(uint8_t type)
{
  return eCWSTARTSMART(type);
}

////////////////////////////////////////

bool ESP8266_stopSmartConfig()
{
  return eCWSTOPSMART();
}

////////////////////////////////////////

uint8_t *ESP8266_getIPStatus()
{
  eATCIPSTATUS(ESP32_ap_list);
  return ESP32_ap_list;
}

////////////////////////////////////////

uint8_t *ESP8266_getLocalIP()
{
  eATCIFSR(ESP32_ap_list);
  return ESP32_ap_list;
}

////////////////////////////////////////

bool ESP8266_enableMUX()
{
  return sATCIPMUX(1);
}

////////////////////////////////////////

bool ESP8266_disableMUX()
{
  return sATCIPMUX(0);
}

////////////////////////////////////////

bool ESP8266_createTCP(uint8_t * addr, uint32_t port)
{
  return sATCIPSTARTSingle("TCP", addr, port);
}

////////////////////////////////////////

bool ESP8266_releaseTCP()
{
  return eATCIPCLOSESingle();
}

////////////////////////////////////////

bool ESP8266_registerUDP(uint8_t * addr, uint32_t port)
{
  return sATCIPSTARTSingle("UDP", addr, port);
}

////////////////////////////////////////

bool ESP8266_unregisterUDP()
{
  return eATCIPCLOSESingle();
}

////////////////////////////////////////

bool ESP8266_createTCP_mux(uint8_t mux_id, uint8_t * addr, uint32_t port)
{
  return sATCIPSTARTMultiple(mux_id, "TCP", addr, port);
}

////////////////////////////////////////

bool ESP8266_releaseTCP_mux(uint8_t mux_id)
{
  return sATCIPCLOSEMultiple(mux_id);
}

////////////////////////////////////////

bool ESP8266_registerUDP_mux(uint8_t mux_id, uint8_t * addr, uint32_t port)
{
  return sATCIPSTARTMultiple(mux_id, "UDP", addr, port);
}

////////////////////////////////////////

bool ESP8266_unregisterUDP_mux(uint8_t mux_id)
{
  return sATCIPCLOSEMultiple(mux_id);
}

////////////////////////////////////////

bool ESP8266_setTCPServerTimeout(uint32_t timeout)
{
  return sATCIPSTO(timeout);
}

////////////////////////////////////////

bool ESP8266_startTCPServer(uint32_t port)
{
  if (sATCIPSERVER(1, port))
  {
    return true;
  }

  return false;
}

////////////////////////////////////////

bool ESP8266_stopTCPServer()
{
  sATCIPSERVER(0, 333);
  restart();
  
  return false;
}

////////////////////////////////////////

bool ESP8266_setCIPMODE(uint8_t mode)
{
  return sATCIPMODE(mode);
}

////////////////////////////////////////

bool ESP8266_saveTransLink (uint8_t mode, uint8_t * ip, uint32_t port)
{
  return eATSAVETRANSLINK(mode, ip, port);
}

////////////////////////////////////////

bool ESP8266_setPing(uint8_t * ip)
{
  return eATPING(ip);
}

////////////////////////////////////////

bool ESP8266_startServer(uint32_t port)
{
  return startTCPServer(port);
}

////////////////////////////////////////

bool ESP8266_stopServer()
{
  return stopTCPServer();
}

////////////////////////////////////////

bool ESP8266_send(const uint8_t *buffer, uint32_t len)
{
  return sATCIPSENDSingle(buffer, len);
}

////////////////////////////////////////

bool ESP8266_sendFromFlash_mux(uint8_t mux_id, const uint8_t *buffer, uint32_t len)
{
  return sATCIPSENDMultipleFromFlash(mux_id, buffer, len);
}

////////////////////////////////////////

bool ESP8266_sendFromFlash(const uint8_t *buffer, uint32_t len)
{
  return sATCIPSENDSingleFromFlash(buffer, len);
}

////////////////////////////////////////

bool ESP8266_send_mux(uint8_t mux_id, const uint8_t *buffer, uint32_t len)
{
  return sATCIPSENDMultiple(mux_id, buffer, len);
}

////////////////////////////////////////

void ESP8266_run()
{
  rx_empty();
}

////////////////////////////////////////

/*----------------------------------------------------------------------------*/
/* +IPD,<id>,<len>:<data> */
/* +IPD,<len>:<data> */



static uint32_t checkIPD(uint8_t *data)
{
  //Serial.print("### check: ");
  //Serial.println(data);

  int32_t index_PIPDcomma = -1;
  int32_t index_colon     = -1; /* : */
  int32_t index_comma     = -1; /* , */

  int32_t len = -1;
  int8_t id   = -1;
//printf("checkIPD len=%d\n",strlen(data));
  { // Just for easier diffing
 //   index_PIPDcomma = data.indexOf("+IPD,");
    index_PIPDcomma = indexOfString(data,"+IPD,");
    if (index_PIPDcomma != -1)
    {
      index_colon = indexOfChar(data,':', index_PIPDcomma + 5);

      if (index_colon != -1)
      {
        index_comma = indexOfChar(data,',', index_PIPDcomma + 5);

        /* +IPD,id,len:data */
        if (index_comma != -1 && index_comma < index_colon)
        {
#if 0
          id = data.substring(index_PIPDcomma + 5, index_comma).toInt();
#else
          data[index_comma] = 0;
          id = atoi(data + index_PIPDcomma + 5);
          data[index_comma] = ',';
#endif
          if (id < 0 || id > 4)
          {
            return 0;
          }
#if 0
          len = data.substring(index_comma + 1, index_colon).toInt();
#else
          data[index_colon] = 0;
          len = atoi(data + index_comma + 1);
          data[index_colon] = ':';
#endif
          if (len <= 0)
          {
            return 0;
          }
        }
        else
        {
          /* +IPD,len:data */
#if 0
          len = data.substring(index_PIPDcomma + 5, index_colon).toInt();
#else
          data[index_colon] = 0;
          len = atoi(data + index_PIPDcomma + 5);
          data[index_colon] = ':';
#endif
          if (len <= 0)
          {
            return 0;
          }
        }

#if 0 // TODO by isjeon
        if (m_onData)
        {
          m_onData(id, len, m_onDataPtr);
        }
#endif
        return len;
      }
    }
  }

  return 0;
}

////////////////////////////////////////

void rx_empty()
{
//  uint8_t data[1024];
  uint8_t ch;
  int idx = 0;
  char a;
  unsigned long start = millis();

  while (millis() - start < 10)
  {
    if (m_puart->available())
    {
      a = m_puart->read();
#if 0
      if (a == '\0')
        continue;

//      data += a;
      data[idx++] = a;
      if(idx >= 1024)
      {
    	  idx = 0;
    	  memset(data,0,sizeof(data));
      }
      if (checkIPD(data))
      {
//        data = "";
    	  idx = 0;
    	  memset(data,0,sizeof(data));
      }
#endif //by isjeon
      start = millis();
    }
  }
}

////////////////////////////////////////
//static
uint8_t data[3*1024];
char *recvString(uint8_t *target)
{

  int idx = 0;
  char a;
  uint32_t timeout = 1000; //default 1 secons by isjeon
  unsigned long start = millis();
  memset(data,0,sizeof(data));

  while (millis() - start < timeout)
  {
    while (m_puart->available() > 0)
    {
      a = m_puart->read();

      if (a == '\0')
        continue;

//      data += a;
      data[idx++] = a;

      if (strstr(data,target))
      {
        return data;
      }
      else if (checkIPD(data))
      {
    	memset(data,0,sizeof(data));
 //       data = "";
      }
    }
  }

  return data;
}
char *recvString1_TO(uint8_t *target, uint32_t timeout)
{

  char a;
  unsigned long start = millis();
  int idx = 0;
  int tick = -1;

#if 1
  memset(data,0,sizeof(data));
  while (millis() - start < timeout)
  {
    while (m_puart->available() > 0)
    {
      a = m_puart->read();

      if (a == '\0')
        continue;

//      data += a;

      if(idx >= sizeof(data) - 1)
      {
    	  printf("Too long size = %d\n",idx);
      }
      else
      {
          data[idx++] = a;
      }
      if (strstr(data,target))
      {
        return data;
      }
      else if (checkIPD(data))
      {
    	memset(data,0,sizeof(data));
 //       data = "";
      }
    }
  }
#else
  tick = -1;
  while(millis() - start < timeout)
  {
	  if (!m_puart->available()) {
		  delay(1);
		  if(tick >= 0)
		  {
			  tick ++;
			  if(tick >= 10000) break;
		  }
		  continue;
	  }
      a = m_puart->read();
      tick = 0;
      if (a == '\0')
        continue;
      if(idx >= sizeof(data) - 1)
      {
    	  printf("Too long size = %d\n",idx);
      }
      else
      {
          data[idx++] = a;
      }
  }
  data[idx] = 0;
  if (strstr(data,target))
  {
    return data;
  }
  else if (checkIPD(data))
  {
	 data[0] = 0;
  }
#endif

  return data;
}

////////////////////////////////////////

uint8_t * recvString2_TO(uint8_t * target1, uint8_t * target2, uint32_t timeout)
{
  char a;
  unsigned long start = millis();
  int idx = 0;
  memset(data,0,sizeof(data));

  while (millis() - start < timeout)
  {
    while (m_puart->available() > 0)
    {
      a = m_puart->read();

      if (a == '\0')
        continue;

 //     data += a;
      data[idx++] = a;
//      if (data.indexOf(target1) != -1)
      if (strstr(data,target1))
      {
        return data;
      }
      else if (strstr(data,target2))
      {
        return data;
      }
      else if (checkIPD(data))
      {
//        data = "";
        idx = 0;
        memset(data,0,sizeof(data));
      }
    }
  }

  return data;
}

////////////////////////////////////////

uint8_t *recvString3_TO(uint8_t * target1, uint8_t * target2, uint8_t * target3, uint32_t timeout)
{
  char a;
  unsigned long start = millis();
  int idx = 0;
  memset(data,0,sizeof(data));
  while (millis() - start < timeout)
  {
    while (m_puart->available() > 0)
    {
      a = m_puart->read();

      if (a == '\0')
        continue;

 //     data += a;
      data[idx++] = a;
      if (strstr(data,target1))
      {
        return data;
      }
      else if (strstr(data,target2))
      {
        return data;
      }
      else if (strstr(data,target3))
      {
        return data;
      }
      else if (checkIPD(data))
      {
//        data = "";
        idx = 0;
        memset(data,0,sizeof(data));
      }
    }
  }
  return data;
}

////////////////////////////////////////

bool recvFind(uint8_t * target, uint32_t timeout)
{
  char *data_tmp;

  data_tmp = recvString1_TO(target, timeout);

  if (indexOfString(data_tmp,target) != -1)
  {
    return true;
  }

  return false;
}

////////////////////////////////////////

bool recvFindAndFilter(uint8_t * target, uint8_t * begin, uint8_t * end, uint8_t *data, uint32_t timeout)
{
  uint8_t * data_tmp;

  data_tmp = recvString1_TO(target, timeout);
/*
 * by isjeon for test
 */
//  printf("Rx str = %s\n",data_tmp);
  if(0){
//#include <stdio.h>
   extern int _write(int file, char *data, int len);
   printf("Len = %d\n",strlen(data_tmp));
   _write(0, data_tmp, strlen(data_tmp));
   printf("[***]\n");
//   while(1);
  }
  if (indexOfString(data_tmp,target) != -1)
  {
    int32_t index1 = indexOfString(data_tmp,begin);
    int32_t index2 = indexOfString(data_tmp,end);

    if (index1 != -1 && index2 != -1)
    {
      index1 += strlen(begin); // begin.length();
#if 0 //by isjeon
      data = data_tmp.substring(index1, index2);
#else
      strncpy(data,data_tmp + index1, index2 - index1);
      data[index2 - index1] = 0;
#endif
      return true;
    }
    else if (index2 != -1)
    {
#if 0
      data = data_tmp.substring(0, index2);
#else
      strncpy(data, data_tmp + 0, index2 - 0);
      data[index2 - 0] = 0;
#endif
      return true;
    }

#if 0
    index1 = data.indexOf("\r\n\r\nOK");
#else
    index1 = indexOfString(data,"\r\n\r\nOK");
#endif
    if (index1 != -1)
    {
#if 0
      data = data_tmp.substring(0, index1);
#else
      strncpy(data, data_tmp + 0, index1);
      data[index1] = 0;
#endif
    }

    index1 = indexOfString(data, "\r\nOK");

    if (index1 != -1)
    {
#if 0
      data = data_tmp.substring(0, index1);
#else
      strncpy(data, data_tmp + 0, index1);
      data[index1] = 0;
#endif
    }
  }

#if 0
  data = data_tmp;
#else
  strcpy(data,data_tmp); // 이상함?  위 OK 관련 파싱하는 부분이 의미가 없는 코드??
#endif
  return false;
}

////////////////////////////////////////
////////////////////////////////////////

bool eAT()
{
  rx_empty();
  m_puart->println(F("AT"));

  AT_LIB_LOGDEBUG(F("AT"));

  return recvFind("OK",1000);
}

////////////////////////////////////////

bool eATRST()
{
  rx_empty();

#if 0   //USE_ESP32_AT
  //The execution of this command will reset all parameters saved in flash, and restore the factory default settings of the module.
  // The chip will be restarted when this command is executed.
  m_puart->println(F("AT+RESTORE"));

  AT_LIB_LOGDEBUG(F("AT+RESTORE"));
  
  //return recvFind("OK");
  return recvFind("OK", 5000);
#else
  m_puart->println(F("AT+RST"));

  AT_LIB_LOGDEBUG(F("AT+RST"));
  
  return recvFind("OK",1000);
#endif
}

////////////////////////////////////////

bool eATGMR(char *version)
{
  rx_empty();
  delay(3000);
  m_puart->println(F("AT+GMR"));

  AT_LIB_LOGDEBUG(F("AT+GMR"));

  return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", version, 10000);
}

////////////////////////////////////////

// Enters Deep-sleep Mode in time
bool eATGSLP(uint32_t time)
{
  rx_empty();
  m_puart->print(F("AT+GSLP="));
  m_puart->println_int(time);

  AT_LIB_LOGDEBUG(F("AT+GSLP="));

  return recvFind("OK",1000);
}

////////////////////////////////////////

// AT Commands Echoing (mode = 0: OFF, 1: ON)
bool eATE(uint8_t mode)
{
  rx_empty();
  m_puart->print(F("ATE"));
  m_puart->println(mode);

  AT_LIB_LOGDEBUG(F("ATE"));

  return recvFind("OK",1000);
}

////////////////////////////////////////

// Restores the Factory Default Settings
bool eATRESTORE()
{
  rx_empty();
  m_puart->println(F("AT+RESTORE"));

  AT_LIB_LOGDEBUG(F("AT+RESTORE"));

  return recvFind("OK",1000);
}

////////////////////////////////////////

bool eATSETUART(uint32_t baudrate, uint8_t pattern)
{
  rx_empty();

  if (pattern > 3 || pattern < 1)
  {
    return false;
  }

  // ESP32-AT supports all _CUR, _DEF
  // AT+UART_DEF=<baudrate>,<databits>,<stopbits>,<parity>,<flow control>
  // AT+UART_CUR=115200,8,1,0,0
  switch (pattern)
  {
    case 1:
    // Deprecated => using _CUR
    //m_puart->print(F("AT+UART="));
    //AT_LIB_LOGDEBUG(F("AT+UART="));
    //break;
    case 2:
      m_puart->print(F("AT+UART_CUR="));
      AT_LIB_LOGDEBUG(F("AT+UART_CUR="));
      break;
    case 3:
      m_puart->print(F("AT+UART_DEF="));
      AT_LIB_LOGDEBUG(F("AT+UART_DEF="));
      break;
  }

  m_puart->print_int(baudrate);
  m_puart->print(F(","));
  m_puart->print_int(8);
  m_puart->print(F(","));
  m_puart->print_int(1);
  m_puart->print(F(","));
  m_puart->print_int(0);
  m_puart->print(F(","));
  m_puart->println_int(0);

  if (recvFind("OK", 5000))
  {
    return true;
  }
  else
  {
    return false;
  }
}

////////////////////////////////////////

// Get the Wi-Fi Mode (Station/SoftAP/Station+SoftAP)
// ESP32-AT not support _CUR and _DEF here
bool qATCWMODE(uint8_t *mode, uint8_t pattern)
{
  uint8_t str_mode[128];
  bool ret;

  if (!mode || !pattern)
  {
    return false;
  }
  rx_empty();

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->println(F("AT+CWMODE_DEF?"));
      AT_LIB_LOGDEBUG(F("AT+CWMODE_DEF?"));
      break;
    case 2:
      m_puart->println(F("AT+CWMODE_CUR?"));
      AT_LIB_LOGDEBUG(F("AT+CWMODE_CUR?"));
      break;
#endif
    default:
      m_puart->println(F("AT+CWMODE?"));
      AT_LIB_LOGDEBUG(F("AT+CWMODE?"));
      break;
  }

  ret = recvFindAndFilter("OK", ":", "\r\n\r\nOK", str_mode, 1000);

  if (ret)
  {
//    *mode = (uint8_t)str_mode.toInt();
	*mode = atoi(str_mode);
    return true;
  }
  else
  {
    return false;
  }
}

////////////////////////////////////////

// Test if the Wi-Fi Mode (Station/SoftAP/Station+SoftAP) supported
bool eATCWMODE(uint8_t *list)
{
  rx_empty();
  m_puart->println(F("AT+CWMODE=?"));

  AT_LIB_LOGDEBUG(F("AT+CWMODE=?"));

  return recvFindAndFilter("OK", "+CWMODE:(", ")\r\n\r\nOK", list, 1000);
}

////////////////////////////////////////

// Set the Wi-Fi Mode (Station/SoftAP/Station+SoftAP)
// ESP32-AT not support _CUR and _DEF here
bool sATCWMODE(uint8_t mode, uint8_t pattern)
{
  if (!pattern)
  {
    return false;
  }

  uint8_t *data;

  rx_empty();

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->print(F("AT+CWMODE_DEF="));
      AT_LIB_LOGDEBUG(F("AT+CWMODE_DEF="));
      break;
    case 2:
      m_puart->print(F("AT+CWMODE_CUR="));
      AT_LIB_LOGDEBUG(F("AT+CWMODE_CUR="));
      break;
#endif

#if (USING_WIZFI360) || defined(ARDUINO_WIZNET_WIZFI360_EVB_PICO)
    default:
      m_puart->print(F("AT+CWMODE_DEF="));
      AT_LIB_LOGDEBUG(F("AT+CWMODE_DEF="));
      break;
#else
    default:
      m_puart->print(F("AT+CWMODE="));
      AT_LIB_LOGDEBUG(F("AT+CWMODE="));
      break;
#endif
  }

  m_puart->println_int(mode);
  data = recvString2_TO("OK", "no change", 1000);

#if 0 //by isjeon
  if (data.indexOf("OK") != -1 || data.indexOf("no change") != -1)
  {
    return true;
  }
#else
  if(data && (strstr(data,"OK") || strstr(data, "no change")))
  {
	  return true;
  }
#endif
  return false;
}

////////////////////////////////////////

// Get connected AP info
// ESP32-AT not support _CUR and _DEF here
bool qATCWJAP(uint8_t *ssid, uint8_t pattern)
{
  //bool ret;

  if (!pattern)
  {
    return false;
  }

  rx_empty();

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->println(F("AT+CWJAP_DEF?"));
      AT_LIB_LOGDEBUG(F("AT+CWJAP_DEF?"));
      break;
    case 2:
      m_puart->println(F("AT+CWJAP_CUR?"));
      AT_LIB_LOGDEBUG(F("AT+CWJAP_CUR?"));
      break;
#endif
    default:
      m_puart->println(F("AT+CWJAP?"));
      AT_LIB_LOGDEBUG(F("AT+CWJAP?"));
      break;
  }

#if 0
  ssid = recvString("OK", "No AP");

  if (ssid.indexOf("OK") != -1 || ssid.indexOf("No AP") != -1)
  {
    return true;
  }
#else
  strcpy(ssid, recvString2_TO("OK", "No AP",1000));
  if (strstr(ssid,"OK") || strstr(ssid,"No AP"))
  {
    return true;
  }
#endif
  return false;
}

////////////////////////////////////////

// Connects to an AP
// ESP32-AT not support _CUR and _DEF here
bool sATCWJAP(uint8_t *ssid, uint8_t *pwd, uint8_t pattern)
{
  uint8_t data[256];

  if (!pattern)
  {
    return false;
  }

  rx_empty();

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->print(F("AT+CWJAP_DEF=\""));
      AT_LIB_LOGDEBUG(F("AT+CWJAP_DEF=\""));
      break;
    case 2:
      m_puart->print(F("AT+CWJAP_CUR=\""));
      AT_LIB_LOGDEBUG(F("AT+CWJAP_CUR=\""));
      break;
#endif

#if (USING_WIZFI360) || defined(ARDUINO_WIZNET_WIZFI360_EVB_PICO)
    default:
      m_puart->print(F("AT+CWJAP_DEF=\""));
      AT_LIB_LOGDEBUG(F("AT+CWJAP_DEF=\""));
      break;
#else
    default:
      m_puart->print(F("AT+CWJAP=\""));
      AT_LIB_LOGDEBUG(F("AT+CWJAP=\""));
      break;
#endif      
  }

  m_puart->print(ssid);
  m_puart->print(F("\",\""));
  m_puart->print(pwd);
  m_puart->println(F("\""));

#if USE_ESP32_AT
  strcpy(data,recvString2_TO("OK", "ERROR", 10000));
#else
  data = recvString("OK", "FAIL", 10000);
#endif

#if 0
  if (data.indexOf("OK") != -1)
#else
  if(strstr(data,"OK"))
#endif
  {
    return true;
  }

  return false;
}

////////////////////////////////////////

// Lists the Available APs
bool eATCWLAP(uint8_t *list)
{
//  uint8_t data[256];
  rx_empty();
  m_puart->println(F("AT+CWLAP"));

  AT_LIB_LOGDEBUG(F("AT+CWLAP"));

  return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list, 15 * 1000);
}

////////////////////////////////////////

// Disconnects from the AP
bool eATCWQAP()
{
// String data;

  rx_empty();
  m_puart->println(F("AT+CWQAP"));

  AT_LIB_LOGDEBUG(F("AT+CWQAP"));

  return recvFind("OK",1000);
}

////////////////////////////////////////

// Get Configuration Params of the ESP8266/ESP32 SoftAP
// ESP32-AT not support _CUR and _DEF here
bool qATCWSAP(uint8_t *List, uint8_t pattern)
{
  if (!pattern)
  {
    return false;
  }

  rx_empty();

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->println(F("AT+CWSAP_DEF?"));
      AT_LIB_LOGDEBUG(F("AT+CWSAP_DEF?"));
      break;
    case 2:
      m_puart->println(F("AT+CWSAP_CUR?"));
      AT_LIB_LOGDEBUG(F("AT+CWSAP_CUR?"));
      break;
#endif
    default:
      m_puart->println(F("AT+CWSAP?"));
      AT_LIB_LOGDEBUG(F("AT+CWSAP?"));
      break;
  }

  return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", List, 10000);
}

////////////////////////////////////////

// Set Configuration Params of the ESP8266/ESP32 SoftAP
// ESP32-AT not support _CUR and _DEF here
bool sATCWSAP(uint8_t *ssid, uint8_t *pwd, uint8_t chl, uint8_t ecn, uint8_t pattern)
{
  uint8_t *data;

  if (!pattern)
  {
    return false;
  }

  rx_empty();

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->print(F("AT+CWSAP_DEF=\""));
      AT_LIB_LOGDEBUG(F("AT+CWSAP_DEF=\""));
      break;
    case 2:
      m_puart->print(F("AT+CWSAP_CUR=\""));
      AT_LIB_LOGDEBUG(F("AT+CWSAP_CUR=\""));
      break;
#endif
    default:
      m_puart->print(F("AT+CWSAP=\""));
      AT_LIB_LOGDEBUG(F("AT+CWSAP=\""));
      break;
  }

  m_puart->print(ssid);
  m_puart->print(F("\",\""));
  m_puart->print(pwd);
  m_puart->print(F("\","));
  m_puart->print_int(chl);
  m_puart->print(F(","));
  m_puart->println_int(ecn);

#if USE_ESP32_AT
  data = recvString1_TO("OK", 5000);
#else
  data = recvString("OK", "ERROR", 5000);
#endif

  if (indexOfString(data, "OK") != -1)
  {
    return true;
  }

  return false;
}

////////////////////////////////////////

// Get IP of Stations to Which the ESP8266/ESP32 SoftAP is connected
bool eATCWLIF(uint8_t *list)
{
//  String data;

  rx_empty();
  m_puart->println(F("AT+CWLIF"));

  AT_LIB_LOGDEBUG(F("AT+CWLIF"));

  return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list, 1000);
}

////////////////////////////////////////

// Get info if DHCP Enabled or Disabled
// ESP32-AT not support _CUR and _DEF here and has different command format
bool qATCWDHCP(uint8_t *List, uint8_t pattern)
{
  if (!pattern)
  {
    return false;
  }

  rx_empty();

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->println(F("AT+CWDHCP_DEF?"));
      AT_LIB_LOGDEBUG(F("AT+CWDHCP_DEF?"));
      break;
    case 2:
      m_puart->println(F("AT+CWDHCP_CUR?"));
      AT_LIB_LOGDEBUG(F("AT+CWDHCP_CUR?"));
      break;
#endif
    default:
      m_puart->println(F("AT+CWDHCP?"));
      AT_LIB_LOGDEBUG(F("AT+CWDHCP?"));
      break;
  }

  return recvFindAndFilter("OK", "\r\r\n", "\r\nOK", List, 10000);
}

////////////////////////////////////////

// Get info if DHCP Enabled or Disabled
// ESP32-AT not support _CUR and _DEF here and has different command format
// Be careful
// ESP32-AT => AT+CWDHCP=<operate>,<mode>
// • <operate>:
//    ‣ 0: disable
//    ‣ 1: enable
// • <mode>:
//    ‣ Bit0: Station DHCP
//    ‣ Bit1: SoftAP DHCP
//
// ESP8266-AT => AT+CWDHCP=<mode>,<en>
// • <mode>:
//    ‣ 0: Sets ESP8266 SoftAP
//    ‣ 1: Sets ESP8266 Station
//    ‣ 2: Sets both SoftAP and Station
// • <en>:
//    ‣ 0: disable DHCP
//    ‣ 1: enable DHCP

////////////////////////////////////////

#if USE_ESP32_AT
// For ESP32-AT
// ESP32-AT => AT+CWDHCP=<operate>,<mode>
// • <operate>:
//    ‣ 0: disable
//    ‣ 1: enable
// • <mode>:
//    ‣ Bit0: Station DHCP
//    ‣ Bit1: SoftAP DHCP
bool sATCWDHCP(uint8_t op, uint8_t mode, uint8_t pattern)
{
   uint8_t *data;

  if (!pattern)
  {
    return false;
  }

  rx_empty();

  switch (pattern)
  {
    default:
      m_puart->print(F("AT+CWDHCP="));
      AT_LIB_LOGDEBUG(F("AT+CWDHCP="));
  }

  m_puart->print_int(op);
  m_puart->print(F(","));
  m_puart->println_int(mode);
  data = recvString1_TO("OK", 2000);

  if (indexOfString(data, "OK") != -1)
  {
    return true;
  }

  return false;
}

////////////////////////////////////////

#else

////////////////////////////////////////

// For ESP8266
// ESP8266-AT => AT+CWDHCP=<mode>,<en>
// • <mode>:
//    ‣ 0: Sets ESP8266 SoftAP
//    ‣ 1: Sets ESP8266 Station
//    ‣ 2: Sets both SoftAP and Station
// • <en>:
//    ‣ 0: disable DHCP
//    ‣ 1: enable DHCP
bool sATCWDHCP(uint8_t mode, uint8_t en, uint8_t pattern)
{
  String data;

  if (!pattern)
  {
    return false;
  }

  rx_empty();

  switch (pattern)
  {
    case 1 :
      m_puart->print(F("AT+CWDHCP_DEF="));
      AT_LIB_LOGDEBUG(F("AT+CWDHCP_DEF="));
      break;
    case 2:
      m_puart->print(F("AT+CWDHCP_CUR="));
      AT_LIB_LOGDEBUG(F("AT+CWDHCP_CUR="));
      break;
    default:
      m_puart->print(F("AT+CWDHCP="));
      AT_LIB_LOGDEBUG(F("AT+CWDHCP="));
  }

  m_puart->print(mode);
  m_puart->print(F(","));
  m_puart->println(en);
  data = recvString("OK", "ERROR", 2000);

  if (data.indexOf("OK") != -1)
  {
    return true;
  }

  return false;
}

////////////////////////////////////////

#endif

////////////////////////////////////////

// ESP8266-AT => AT+CWDHCP=<mode>,<en>
// • <en>:
//    ‣ 0: disable DHCP
//    ‣ 1: enable DHCP
// Auto-Connects to the AP or Not
bool eATCWAUTOCONN(uint8_t en)
{
  rx_empty();

  if (en > 1)
  {
    return false;
  }

  m_puart->print(F("AT+CWAUTOCONN="));

  AT_LIB_LOGDEBUG(F("AT+CWAUTOCONN="));

  m_puart->println_int(en);

  return recvFind("OK",1000);
}

////////////////////////////////////////

// Get the MAC Address of the ESP8266/ESP32 Station
// ESP32-AT not support _CUR and _DEF here
bool qATCIPSTAMAC(uint8_t *mac, uint8_t pattern)
{
  rx_empty();

  if (!pattern)
  {
    return false;
  }

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->println(F("AT+CIPSTAMAC_DEF?"));
      AT_LIB_LOGDEBUG(F("AT+CIPSTAMAC_DEF?"));
      break;
    case 2:
      m_puart->println(F("AT+CIPSTAMAC_CUR?"));
      AT_LIB_LOGDEBUG(F("AT+CIPSTAMAC_CUR?"));
      break;
#endif
    default:
      m_puart->println(F("AT+CIPSTAMAC?"));
      AT_LIB_LOGDEBUG(F("AT+CIPSTAMAC?"));
      break;
  }

  return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", mac, 2000);
}

////////////////////////////////////////

// Set the MAC Address of the ESP8266/ESP32 Station
// ESP32-AT not support _CUR and _DEF here
bool eATCIPSTAMAC(uint8_t * mac, uint8_t pattern)
{
  rx_empty();

  if (!pattern)
  {
    return false;
  }

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->print(F("AT+CIPSTAMAC_DEF="));
      AT_LIB_LOGDEBUG(F("AT+CIPSTAMAC_DEF="));
      break;
    case 2:
      m_puart->print(F("AT+CIPSTAMAC_CUR="));
      AT_LIB_LOGDEBUG(F("AT+CIPSTAMAC_CUR="));
      break;
#endif
    default:
      m_puart->print(F("AT+CIPSTAMAC="));
      AT_LIB_LOGDEBUG(F("AT+CIPSTAMAC="));
      break;
  }

  m_puart->print(F("\""));
  m_puart->print(mac);
  m_puart->println(F("\""));

  return recvFind("OK",1000);
}

////////////////////////////////////////

// Get the IP Address of the ESP8266/ESP32 Station
// ESP32-AT not support _CUR and _DEF here
bool qATCIPSTAIP(uint8_t *ip, uint8_t pattern)
{
  rx_empty();

  if (!pattern)
  {
    return false;
  }

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->println(F("AT+CIPSTA_DEF?"));
      AT_LIB_LOGDEBUG(F("AT+CIPSTA_DEF?"));
      break;
    case 2:
      m_puart->println(F("AT+CIPSTA_CUR?"));
      AT_LIB_LOGDEBUG(F("AT+CIPSTA_CUR?"));
      break;
#endif
    default:
      m_puart->println(F("AT+CIPSTA?"));
      AT_LIB_LOGDEBUG(F("AT+CIPSTA?"));
      break;
  }

  return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", ip, 2000);
}

////////////////////////////////////////

// Set the IP Address of the ESP8266/ESP32 Station
// ESP32-AT not support _CUR and _DEF here
bool eATCIPSTAIP(uint8_t * ip, uint8_t * gateway, uint8_t * netmask, uint8_t pattern)
{
  rx_empty();

  if (!pattern)
  {
    return false;
  }

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->print(F("AT+CIPSTA_DEF="));
      AT_LIB_LOGDEBUG(F("AT+CIPSTA_DEF="));
      break;
    case 2:
      m_puart->print(F("AT+CIPSTA_CUR="));
      AT_LIB_LOGDEBUG(F("AT+CIPSTA_CUR="));
      break;
#endif
    default:
      m_puart->print(F("AT+CIPSTA="));
      AT_LIB_LOGDEBUG(F("AT+CIPSTA="));
      break;
  }

  m_puart->print(F("\""));
  m_puart->print(ip);
  m_puart->print(F("\",\""));
  m_puart->print(gateway);
  m_puart->print(F("\",\""));
  m_puart->print(netmask);
  m_puart->println(F("\""));

  return recvFind("OK",1000);
}

////////////////////////////////////////

// Get the IP Address of the ESP8266/ESP32 SoftAP
// ESP32-AT not support _CUR and _DEF here
bool qATCIPAP(uint8_t *ip, uint8_t pattern)
{
  rx_empty();

  if (!pattern)
  {
    return false;
  }

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->println(F("AT+CIPAP_DEF?"));
      AT_LIB_LOGDEBUG(F("AT+CIPAP_DEF?"));
      break;
    case 2:
      m_puart->println(F("AT+CIPAP_CUR?"));
      AT_LIB_LOGDEBUG(F("AT+CIPAP_CUR?"));
      break;
#endif
    default:
      m_puart->println(F("AT+CIPAP?"));
      AT_LIB_LOGDEBUG(F("AT+CIPAP?"));
      break;
  }

  return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", ip, 2000);
}

////////////////////////////////////////

// Set the IP Address of the ESP8266/ESP32 SoftAP
// ESP32-AT not support _CUR and _DEF here
bool eATCIPAP(uint8_t * ip, uint8_t pattern)
{
  rx_empty();

  if (!pattern)
  {
    return false;
  }

  switch (pattern)
  {
#if !USE_ESP32_AT
    case 1 :
      m_puart->print(F("AT+CIPAP_DEF="));
      AT_LIB_LOGDEBUG(F("AT+CIPAP_DEF="));
      break;
    case 2:
      m_puart->print(F("AT+CIPAP_CUR="));
      AT_LIB_LOGDEBUG(F("AT+CIPAP_CUR="));
      break;
#endif
    default:
      m_puart->print(F("AT+CIPAP="));
      AT_LIB_LOGDEBUG(F("AT+CIPAP="));
      break;
  }

  m_puart->print(F("\""));
  m_puart->print(ip);
  m_puart->println(F("\""));

  return recvFind("OK",1000);
}

////////////////////////////////////////

// Starts SmartConfig
// <type>:
// ‣ 1: ESP-TOUCH
// ‣ 2: AirKiss
// ‣ 3: ESP-TOUCH+AirKiss
bool eCWSTARTSMART(uint8_t type)
{
  rx_empty();
  m_puart->print(F("AT+CWSTARTSMART="));

  AT_LIB_LOGDEBUG(F("AT+CWSTARTSMART="));
  m_puart->println_int(type);

  return recvFind("OK",1000);
}

////////////////////////////////////////

// Stops SmartConfig
bool eCWSTOPSMART()
{
  rx_empty();
  m_puart->println(F("AT+CWSTOPSMART"));

  AT_LIB_LOGDEBUG(F("AT+CWSTOPSMART"));

  return recvFind("OK",1000);
}

////////////////////////////////////////

// Gets the Connection Status
bool eATCIPSTATUS(uint8_t *list)
{
//  String data;

  delay(100);
  rx_empty();
  m_puart->println(F("AT+CIPSTATUS"));

  AT_LIB_LOGDEBUG(F("AT+CIPSTATUS"));

  return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list, 1000);
}

////////////////////////////////////////

// Establishes TCP Connection, UDP Transmission or SSL Connection
// Single connection (AT+CIPMUX=0)
// For ESP8266 SSL
// SSL connection needs a large amount of memory; otherwise, it may cause system reboot. Use
// AT+CIPSSLSIZE=<size> to enlarge the SSL buffer size.
bool sATCIPSTARTSingle(uint8_t * type, uint8_t * addr, uint32_t port)
{
  uint8_t * data;

  rx_empty();
  m_puart->print(F("AT+CIPSTART=\""));

  AT_LIB_LOGDEBUG(F("AT+CIPSTART=\""));

  m_puart->print(type);
  m_puart->print(F("\",\""));
  m_puart->print(addr);
  m_puart->print(F("\","));
  m_puart->println_int(port);

#if USE_ESP32_AT
  data = recvString1_TO("OK", 10000);
  
  AT_LIB_LOGDEBUG1("data=", data);

  if (indexOfString(data, "OK") != -1)
  {
    return true;
  }
#else
  data = recvString("OK", "ERROR", "ALREADY CONNECT", 10000);

  if (data.indexOf("OK") != -1 || data.indexOf("ALREADY CONNECT") != -1)
  {
    return true;
  }
#endif

  return false;
}

////////////////////////////////////////

// Establishes TCP Connection, UDP Transmission or SSL Connection
// Multiple Connections (AT+CIPMUX=1)
// For ESP8266 SSL
// SSL connection needs a large amount of memory; otherwise, it may cause system reboot. Use
// AT+CIPSSLSIZE=<size> to enlarge the SSL buffer size.
bool sATCIPSTARTMultiple(uint8_t mux_id, uint8_t * type, uint8_t * addr, uint32_t port)
{
  uint8_t * data;

  rx_empty();
  m_puart->print(F("AT+CIPSTART="));

  AT_LIB_LOGDEBUG(F("AT+CIPSTART="));

  m_puart->print_int(mux_id);
  m_puart->print(F(",\""));
  m_puart->print(type);
  m_puart->print(F("\",\""));
  m_puart->print(addr);
  m_puart->print(F("\","));
  m_puart->println_int(port);

#if USE_ESP32_AT
  data = recvString1_TO("OK", 10000);
  if( indexOfString(data, "OK"))
//  if (data.indexOf("OK") != -1)
  {
    return true;
  }
#else
  data = recvString("OK", "ERROR", "ALREADY CONNECT", 10000);

  if (data.indexOf("OK") != -1 || data.indexOf("ALREADY CONNECT") != -1)
  {
    return true;
  }
#endif

  return false;
}

////////////////////////////////////////

// Sends Data of designated length.
// Single connection: (+CIPMUX=0)     => AT+CIPSEND=<length>
bool sATCIPSENDSingle(const uint8_t *buffer, uint32_t len)
{
  rx_empty();
  m_puart->print(F("AT+CIPSEND="));

  AT_LIB_LOGDEBUG(F("AT+CIPSEND="));

  m_puart->println_int(len);

  if (recvFind(">", 5000))
  {
    rx_empty();
#if 0
    for (uint32_t i = 0; i < len; i++)
    {
      m_puart->write(buffer[i]);
    }
#else
    m_puart->write(buffer,len);
#endif
    return recvFind("SEND OK", 10000);
  }

  return false;
}

////////////////////////////////////////

// Sends Data of designated length.
// Multiple connections: (+CIPMUX=1)  => AT+CIPSEND=<link ID>,<length>
bool sATCIPSENDMultiple(uint8_t mux_id, const uint8_t *buffer, uint32_t len)
{
  rx_empty();
  m_puart->print(F("AT+CIPSEND="));

  AT_LIB_LOGDEBUG(F("AT+CIPSEND="));

  m_puart->print_int(mux_id);
  m_puart->print(F(","));
  m_puart->println_int(len);

  if (recvFind(">", 5000))
  {
    rx_empty();

#if 0 //by isjeon
    for (uint32_t i = 0; i < len; i++)
    {
      m_puart->write(buffer[i]);
    }
#else
     m_puart->write(buffer,len);
#endif
    return recvFind("SEND OK", 10000);
  }

  return false;
}

////////////////////////////////////////

// Sends Data of designated length.
// Single connection: (+CIPMUX=0)     => AT+CIPSEND=<length>
bool sATCIPSENDSingleFromFlash(const uint8_t *buffer, uint32_t len)
{
  rx_empty();
  m_puart->print(F("AT+CIPSEND="));

  AT_LIB_LOGDEBUG(F("AT+CIPSEND="));

  m_puart->println_int(len);

  if (recvFind(">", 5000))
  {
    rx_empty();
#if 0
    for (uint32_t i = 0; i < len; i++)
    {
//      m_puart->write((char) pgm_read_byte(&buffer[i]));
    	m_puart->write((char) buffer[i]);
    }
#else
	m_puart->write(buffer, len);
#endif
    return recvFind("SEND OK", 10000);
  }
  return false;
}

////////////////////////////////////////

// Sends Data of designated length.
// Multiple connections: (+CIPMUX=1)  => AT+CIPSEND=<link ID>,<length>
bool sATCIPSENDMultipleFromFlash(uint8_t mux_id, const uint8_t *buffer, uint32_t len)
{
  rx_empty();
  m_puart->print(F("AT+CIPSEND="));

  AT_LIB_LOGDEBUG(F("AT+CIPSEND="));

  m_puart->print_int(mux_id);
  m_puart->print(F(","));
  m_puart->println_int(len);

  if (recvFind(">", 5000))
  {
    rx_empty();
#if 0
    for (uint32_t i = 0; i < len; i++)
    {
//      m_puart->write((char) pgm_read_byte(&buffer[i]));
    	m_puart->write((char) buffer[i]);
    }
#else
	m_puart->write(buffer, len);
#endif
    return recvFind("SEND OK", 10000);
  }

  return false;
}

////////////////////////////////////////

// Closes TCP/UDP/SSL Connection for multiple connections
// AT+CIPCLOSE=<link ID>
bool sATCIPCLOSEMultiple(uint8_t mux_id)
{
  uint8_t * data;

  rx_empty();
  m_puart->print(F("AT+CIPCLOSE="));

  AT_LIB_LOGDEBUG(F("AT+CIPCLOSE="));

  m_puart->println_int(mux_id);

#if 1
  data = recvString1_TO("OK", 5000);

  if (indexOfString(data, "OK") != -1)
  {
    return true;
  }
#else
  data = recvString("OK", "link is not", 5000);

  if (data.indexOf("OK") != -1 || data.indexOf("link is not") != -1)
  {
    return true;
  }
#endif

  return false;
}

////////////////////////////////////////

// Closes TCP/UDP/SSL Connection for single connections
bool eATCIPCLOSESingle()
{
  rx_empty();
  m_puart->println(F("AT+CIPCLOSE"));

  AT_LIB_LOGDEBUG(F("AT+CIPCLOSE"));

  return recvFind("OK", 5000);
}

////////////////////////////////////////

// Gets the Local IP Address
bool eATCIFSR(uint8_t *list)
{
  rx_empty();
  m_puart->println(F("AT+CIFSR"));

  AT_LIB_LOGDEBUG(F("AT+CIFSR"));

  return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list, 1000);
}

////////////////////////////////////////

// Enables/Disables Multiple Connections
bool sATCIPMUX(uint8_t mode)
{
  uint8_t * data;

  rx_empty();
  m_puart->print(F("AT+CIPMUX="));

  AT_LIB_LOGDEBUG(F("AT+CIPMUX="));

  m_puart->println_int(mode);

  //data = recvString("OK", "Link is builded");
  data = recvString1_TO("OK", 1000);

  if (indexOfString(data, "OK") != -1)
  {
    return true;
  }

  return false;
}

////////////////////////////////////////

// Deletes/Creates TCP Server
// • <mode>:
// ‣ 0: deletes server.
// ‣ 1: creates server.
// • <port>: port number; 333 by default.
// • A TCP server can only be created when multiple connections are activated (AT+CIPMUX=1)
bool sATCIPSERVER(uint8_t mode, uint32_t port)
{
  uint8_t *data;

  if (mode)
  {
    rx_empty();
    m_puart->print(F("AT+CIPSERVER=1,"));

    AT_LIB_LOGDEBUG(F("AT+CIPSERVER=1,"));

    m_puart->println_int(port);

#if 1
    data = recvString1_TO("OK", 1000);

    if (indexOfString(data, "OK") != -1)
    {
      return true;
    }
#else
    data = recvString("OK", "no change");

    if (data.indexOf("OK") != -1 || data.indexOf("no change") != -1)
    {
      return true;
    }
#endif

    return false;
  }
  else
  {
    rx_empty();
    m_puart->println(F("AT+CIPSERVER=0"));

    AT_LIB_LOGDEBUG(F("AT+CIPSERVER=0"));

    return recvFind("\r\r\n", 1000);
  }
}

////////////////////////////////////////

// Sets Transmission Mode
// ‣ 0: normal transmission mode.
// ‣ 1: UART-Wi-Fi passthrough mode (transparent transmission), which can only be enabled in TCP/SSL
//      single connection mode or in UDP mode when the remote IP and port do not change.
bool sATCIPMODE(uint8_t mode)
{
   uint8_t *data;

  if (mode > 1)
  {
    return false;
  }

  rx_empty();
  m_puart->print(F("AT+CIPMODE="));

  AT_LIB_LOGDEBUG(F("AT+CIPMODE="));

  m_puart->println_int(mode);

  //data = recvString("OK", "Link is builded", 2000);
  data = recvString2_TO("OK","Link is builded", 2000);

  if (indexOfString(data,"OK"))
  {
    return true;
  }

  return false;
}

////////////////////////////////////////

// Saves the Transparent Transmission Link in Flash
// • <mode>:
// ‣ 0: ESP8266/ESP32 will NOT enter UART-Wi-Fi passthrough mode on power-up.
// ‣ 1: ESP8266/ESP32 will enter UART-Wi-Fi passthrough mode on power-up.
bool eATSAVETRANSLINK(uint8_t mode, uint8_t * ip, uint32_t port)
{
  uint32_t *data;

  rx_empty();
  m_puart->print(F("AT+SAVETRANSLINK="));

  AT_LIB_LOGDEBUG(F("AT+SAVETRANSLINK="));

  m_puart->print(mode);
  m_puart->print(F(",\""));
  m_puart->print(ip);
  m_puart->print(F("\","));
  m_puart->println_int(port);

  //data = recvString("OK", "ERROR", 2000);
  data = recvString2_TO("OK","ERROR", 2000);

  if (indexOfString(data,"OK") != -1)
  {
    return true;
  }

  return false;
}

////////////////////////////////////////

// Ping Packets
bool eATPING(uint8_t * ip)
{
  rx_empty();
  m_puart->print(F("AT+PING="));

  AT_LIB_LOGDEBUG(F("AT+PING="));

  m_puart->print(F("\""));
  m_puart->print(ip);
  m_puart->println(F("\""));

  return recvFind("OK", 2000);
}

////////////////////////////////////////

// Sets the TCP Server Timeout
bool sATCIPSTO(uint32_t timeout)
{
  rx_empty();
  m_puart->print(F("AT+CIPSTO="));

  AT_LIB_LOGDEBUG(F("AT+CIPSTO="));

  m_puart->println_int(timeout);

  return recvFind("OK", 1000);
}

////////////////////////////////////////
uint32_t recvPkg(uint8_t *buffer, uint32_t buffer_size, uint32_t *data_len, uint32_t timeout, uint8_t *coming_mux_id);
//KH Added
/////////////
uint32_t ESP8266_recv(uint8_t *buffer, uint32_t buffer_size, uint32_t timeout)
{
  return recvPkg(buffer, buffer_size, NULL, timeout, NULL);
}

////////////////////////////////////////

uint32_t ESP8266_recv_mux(uint8_t mux_id, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout)
{
  uint8_t id;
  uint32_t ret;
  
  ret = recvPkg(buffer, buffer_size, NULL, timeout, &id);
  
  if (ret > 0 && id == mux_id) 
  {
    return ret;
  }
  
  return 0;
}

////////////////////////////////////////

uint32_t ESP8266_recv_mux_p(uint8_t *coming_mux_id, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout)
{
  return recvPkg(buffer, buffer_size, NULL, timeout, coming_mux_id);
}

////////////////////////////////////////

/*----------------------------------------------------------------------------*/
/* +IPD,<id>,<len>:<data> */
/* +IPD,<len>:<data> */

uint32_t recvPkg(uint8_t *buffer, uint32_t buffer_size, uint32_t *data_len, uint32_t timeout, uint8_t *coming_mux_id)
{

  char a;
  int32_t index_PIPDcomma = -1;
  int32_t index_colon = -1; /* : */
  int32_t index_comma = -1; /* , */
  int32_t len = -1;
  int8_t id = -1;
  bool has_data = false;
  uint32_t ret;
  unsigned long start;
  uint32_t i;

//by isjeon  uint8_t data[1024];
  int idx = 0;
  memset(data, 0, sizeof(data));
  if (buffer == NULL)
  {
    return 0;
  }

  start = millis();
  
  while (millis() - start < timeout) 
  {
    if (m_puart->available() > 0) 
    {
      a = m_puart->read();
//      data += a;
      data[idx++] = a;
    }

//    index_PIPDcomma = data.indexOf("+IPD,");
    index_PIPDcomma = indexOfString(data,"+IPD,");
    if (index_PIPDcomma != -1) 
    {
 //     index_colon = data.indexOf(':', index_PIPDcomma + 5);
      index_colon = indexOfChar(data, ':', index_PIPDcomma + 5);
      if (index_colon != -1) 
      {
//        index_comma = data.indexOf(',', index_PIPDcomma + 5);
        index_comma = indexOfChar(data,',', index_PIPDcomma + 5);
        /* +IPD,id,len:data */
        if (index_comma != -1 && index_comma < index_colon) 
        {
//          id = data.substring(index_PIPDcomma + 5, index_comma).toInt();
          data[index_comma] = 0;
          id = atoi(data + index_PIPDcomma + 5);
          data[index_comma] = ',';
          if (id < 0 || id > 4) 
          {
            return 0;
          }
          
//          len = data.substring(index_comma + 1, index_colon).toInt();
          data[index_colon] = 0;
          len = atoi(data + index_comma + 1);
          data[index_colon] = ':';
          if (len <= 0) 
          {
            return 0;
          }
        } 
        else 
        { 
          /* +IPD,len:data */
//          len = data.substring(index_PIPDcomma + 5, index_colon).toInt();
          data[index_colon] = 0;
          len = atoi(data + index_PIPDcomma + 5);
          data[index_colon] = ':';
          if (len <= 0) 
          {
            return 0;
          }
        }
        
        has_data = true;
        break;
      }
    }
  }

  if (has_data) 
  {
    i = 0;
    ret = (uint32_t) len > buffer_size ? buffer_size : (uint32_t) len;
    start = millis();
    
    while (millis() - start < 3000) 
    {
      while (m_puart->available() > 0 && i < ret) 
      {
        a = m_puart->read();
        buffer[i++] = a;
      }
      
      if (i == ret) 
      {
        rx_empty();
        
        if (data_len) 
        {
          *data_len = len;
        }
        
        if (index_comma != -1 && coming_mux_id) 
        {
          *coming_mux_id = id;
        }
        
        return ret;
      }
    }
  }
  return 0;
}

////////////////////////////////////////

#endif    // __ESP_AT_LIB_IMPL_H__

