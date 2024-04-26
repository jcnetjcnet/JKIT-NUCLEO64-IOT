/*
 * wiznet_cmd.c
 *
 *  Created on: Feb 1, 2022
 *      Author: isjeon
 */

#include "main.h"
#include "uart.h"
extern int is_available(uart_rx_queue_t *Q);
extern uart_rx_queue_t stdin_uart;

#define PORT_TCPS 5000
#define PORT_UDPS 3000
#define SOCK_TCPS 0
#define SOCK_UDPS 1
#define DATA_BUF_SIZE 2048
#define SOCK_DHCP       2
uint8_t gdata[DATA_BUF_SIZE];
uint8_t dhcpData[DATA_BUF_SIZE];

#include "wizchip_conf.h"
extern int8_t ctlwizchip(ctlwizchip_type cwtype, void* arg);
extern uint8_t DHCP_run(void);
__IO int wiz_started = 0;
#define WITH_DHCP
wiz_NetInfo gWIZNETINFO = { .mac = {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},
#ifdef WITH_DHCP
	.ip = {0,0,0,0},
	.sn = {0,0,0,0},
	.gw = {0,0,0,0},
	.dns = {0,0,0,0},
	.dhcp = NETINFO_DHCP
#else
	.ip = {192,168,0,33},
	.sn = {255,255,255,0},
	.gw = {192,168,0,1},
	.dns = {192,168,0,1},
	.dhcp = NETINFO_STATIC
#endif //NETINFO_STATIC

};
#define _MAIN_DEBUG_
static void Net_Conf()
{
	/* wizchip netconf */
	ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
}

static void Display_Net_Conf()
{
#ifdef _MAIN_DEBUG_
	uint8_t tmpstr[6] = {0,};
#endif

	ctlnetwork(CN_GET_NETINFO, (void*) &gWIZNETINFO);

#ifdef _MAIN_DEBUG_
	// Display Network Information
	ctlwizchip(CW_GET_ID,(void*)tmpstr);

	if(gWIZNETINFO.dhcp == NETINFO_DHCP) printf("\r\n===== %s NET CONF : DHCP =====\r\n",(char*)tmpstr);
		else printf("\r\n===== %s NET CONF : Static =====\r\n",(char*)tmpstr);
	printf(" MAC : %02X:%02X:%02X:%02X:%02X:%02X\r\n", gWIZNETINFO.mac[0], gWIZNETINFO.mac[1], gWIZNETINFO.mac[2], gWIZNETINFO.mac[3], gWIZNETINFO.mac[4], gWIZNETINFO.mac[5]);
	printf(" IP : %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);
	printf(" GW : %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0], gWIZNETINFO.gw[1], gWIZNETINFO.gw[2], gWIZNETINFO.gw[3]);
	printf(" SN : %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0], gWIZNETINFO.sn[1], gWIZNETINFO.sn[2], gWIZNETINFO.sn[3]);
	printf("=======================================\r\n");
#endif
}


void my_ip_assign(void)
{
   getIPfromDHCP(gWIZNETINFO.ip);
   getGWfromDHCP(gWIZNETINFO.gw);
   getSNfromDHCP(gWIZNETINFO.sn);
   getDNSfromDHCP(gWIZNETINFO.dns);
   gWIZNETINFO.dhcp = NETINFO_DHCP;
   /* Network initialization */
   Net_Conf();      // apply from DHCP
#ifdef _MAIN_DEBUG_
   Display_Net_Conf();
   printf("DHCP LEASED TIME : %ld Sec.\r\n", getDHCPLeasetime());
   printf("\r\n");
#endif
}

/************************************
 * @ brief Call back for ip Conflict
 ************************************/
void my_ip_conflict(void)
{
#ifdef _MAIN_DEBUG_
	printf("CONFLICT IP from DHCP\r\n");
#endif
   //halt or reset or any...
   while(1); // this example is halt.
}
#define STATE_DHCP_LEASED 3
extern int8_t   dhcp_state;
void wiz_task()
{
	static uint32_t prev_tick = 0;
	volatile uint32_t cur_tick;
	cur_tick = HAL_GetTick();
	if(dhcp_state == STATE_DHCP_LEASED)
	{
		if(cur_tick - prev_tick < 100) return; // 100 mili dhcp polling
	}
	else if(cur_tick - prev_tick < 10) return; // 10 mili dhcp polling
	prev_tick = cur_tick;
	if(wiz_started)
	{
		int ret;
		if(gWIZNETINFO.dhcp == NETINFO_DHCP)
			ret = DHCP_run();
	}
}

void wiz_fn(int ac, char *av[])
{
	uint8_t fifo_sz[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
	uint8_t temp;
	if(ac >= 2)
	{
		if(!strcmp(av[1],"reset")) {
			if(ctlwizchip(CW_RESET_WIZCHIP, (void *)0) == -1)
			{
				printf("Ethernet reset failed\n");
			}
		}
		else if(!strcmp(av[1],"init"))  {
			if(ctlwizchip(CW_INIT_WIZCHIP,fifo_sz ) == -1)
			{
				printf("Ethernet Init fail\n");
			}
			else
			{
				wizchip_setnetinfo(&gWIZNETINFO);
				if(gWIZNETINFO.dhcp == NETINFO_DHCP)
				{
					DHCP_init(SOCK_DHCP, dhcpData);
					reg_dhcp_cbfunc(my_ip_assign, my_ip_assign, my_ip_conflict);
				}
				wiz_started = 1;

				//dhcp_loop();
			}
		}
		else if(!strcmp(av[1],"hreset"))
		{
			LAN_RST_GPIO_Port->BSRR = LAN_RST_Pin << 16;
			HAL_Delay(200);
			LAN_RST_GPIO_Port->BSRR = LAN_RST_Pin << 0;
		}
		else if(!strcmp(av[1],"version"))
		{
			int ret;
			ret = getVERSIONR();
			printf("ret = %x\n",ret);
		}
#if 0
		else if(!strcmp(av[1],"sreset"))
		{
			ctlwizchip(CW_RESET_PHY, (void *)&temp);
			printf("PHYSTATUS=%x\n",temp);
		}
#endif
		else if(!strcmp(av[1],"stat"))
		{
			ctlwizchip(CW_GET_PHYSTATUS, (void *)&temp);
			printf("PHYSTATUS=%x\n",temp);
		}
		else if(!strcmp(av[1],"lstat"))
		{

			if(ctlwizchip(CW_GET_PHYLINK, (void *)&temp) == -1)
			{
				printf("Unknown link status\n");
			}
			else
			{
				printf("Link status = %d\n",temp);
			}
		}
		else if(!strcmp(av[1],"udp_loop"))
		{
				while(1){
					loopback_udps(SOCK_UDPS, gdata, PORT_UDPS);
			        if(is_available(&stdin_uart)) break;
				}
		}
	}
}
