/*
 * can.c
 *
 *  Created on: Feb 2, 2022
 *      Author: isjeon
 */
#include "main.h"
#include "uart.h"
extern int is_available(uart_rx_queue_t *Q);
extern uart_rx_queue_t stdin_uart;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
void can_start(int chn) // chn 1,2
{
           CAN_FilterTypeDef  sFilterConfig;


           sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
           sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

           sFilterConfig.FilterIdHigh = 0x7FF<<5;
           sFilterConfig.FilterIdLow = 0x0000;
           sFilterConfig.FilterMaskIdHigh = 0;
           sFilterConfig.FilterMaskIdLow = 0;

           sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
           sFilterConfig.FilterActivation = ENABLE;
           sFilterConfig.SlaveStartFilterBank = 14;
           if(chn == 1)
           {
                   sFilterConfig.FilterBank = 0;
                   HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
                   HAL_CAN_Start(&hcan1);
                   HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // | CAN_IT_TX_MAILBOX_EMPTY);
           }
#if 1
           else if(chn == 2)
           {
                   sFilterConfig.FilterBank = 14;
                   HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
                   HAL_CAN_Start(&hcan2);
                   HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
           }
#endif
}

uint32_t g_tx1Cnt = 0;
void can1_loopback()
{
   int i;
   CAN_FilterTypeDef  sFilterConfig;
   static CAN_RxHeaderTypeDef rxHeader; // F1 V1.19.0
   static CAN_TxHeaderTypeDef txHeader;
   static uint8_t txData[8]; // 송신버퍼입니다
   static uint32_t txMailbox;

   memset(txData,0,sizeof(txData));


   while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {} // by isjeon
   txHeader.StdId = 0x555; // 0x7FF;
   txHeader.RTR = CAN_RTR_DATA;
   txHeader.IDE = CAN_ID_STD;
   txHeader.DLC = 8;
   txHeader.TransmitGlobalTime = DISABLE;
   txData[0] = 0xCA;
   txData[1] = 0xFE;
   txData[2] = 0xaa;     txData[3] = 0xaa;     txData[4] = 0xaa;     txData[5] = 0xaa;
//   sprintf(txData+2,"%d",g_tx1Cnt++);
//   HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

   HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);


}
void can1_tx()
{
   int i;
   CAN_FilterTypeDef  sFilterConfig;
   static CAN_RxHeaderTypeDef rxHeader; // F1 V1.19.0
   static CAN_TxHeaderTypeDef txHeader;
   static uint8_t txData[8]; // 송신버퍼입니다
   static uint32_t txMailbox;

   memset(txData,0,sizeof(txData));


   while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {} // by isjeon
   txHeader.StdId = 0x555; // 0x7FF;
   txHeader.RTR = CAN_RTR_DATA;
   txHeader.IDE = CAN_ID_STD;
   txHeader.DLC = 8;
   txHeader.TransmitGlobalTime = DISABLE;
   txData[0] = 0xCA;
   txData[1] = 0xFE;
   txData[2] = 0xaa;     txData[3] = 0xaa;     txData[4] = 0xaa;     txData[5] = 0xaa;
//   sprintf(txData+2,"%d",g_tx1Cnt++);
//   HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

   HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}
void can2_tx()
{
   int i;
   CAN_FilterTypeDef  sFilterConfig;
   static CAN_RxHeaderTypeDef rxHeader; // F1 V1.19.0
   static CAN_TxHeaderTypeDef txHeader;
   static uint8_t txData[8]; // 송신버퍼입니다
   static uint32_t txMailbox;

   memset(txData,0,sizeof(txData));


   while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 3) {} // by isjeon
   txHeader.StdId = 0x555; // 0x7FF;
   txHeader.RTR = CAN_RTR_DATA;
   txHeader.IDE = CAN_ID_STD;
   txHeader.DLC = 8;
   txHeader.TransmitGlobalTime = DISABLE;
   txData[0] = 0xCA;
   txData[1] = 0xFE;
   txData[2] = 0xbb;     txData[3] = 0xbb;     txData[4] = 0xbb;     txData[5] = 0xbb;
//   sprintf(txData+2,"%d",g_tx1Cnt++);
//   HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

   HAL_CAN_AddTxMessage(&hcan2, &txHeader, txData, &txMailbox);
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    printf("\n\rTransmission completed mailbox_0\n\r");
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	printf("\n\rTransmission completed mailbox_1\n\r");

}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	printf("\n\rTransmission completed mailbox_2\n\r");

}
volatile uint8_t rxDataCan1[8];
volatile uint8_t rxDataCan2[8];
volatile int can1_rx_flag = 0;
volatile int can2_rx_flag = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  int i;
   char rxData[8];
   CAN_FilterTypeDef  sFilterConfig;
   static CAN_RxHeaderTypeDef rxHeader; // F1 V1.19.0
   static CAN_TxHeaderTypeDef txHeader;

        if(hcan == &hcan1)
        {
//              printf("Can1 rx \n");
        		can1_rx_flag = 1;
                HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
                HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData);
                memcpy(rxDataCan1, rxData, 8);

//                insert_CAN_if_Q(1,can1_rxData,rxHeader.StdId);

        }
        if(hcan == &hcan2)
        {
//              printf("Can2 rx\n");
    			can2_rx_flag = 1;
                HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0);
                HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rxHeader, rxData);
                memcpy(rxDataCan2, rxData, 8);
//                insert_CAN_if_Q(1,can1_rxData,rxHeader.StdId);

        }

}

void can_fn(int ac, char *av[])
{
	if(ac == 3 && !strcmp(av[1],"tx"))
	{
		int ch;
		ch = av[2][0];
		if(ch == '1')
		{
			printf("CAN1 tx\n");
			can1_tx();
		}
		if(ch == '2')
		{
			printf("CAN2 tx\n");
			can2_tx();
		}
	}
	else
	{
		printf("Usage : can tx [1/2]\n");
	}
#if 0
	while(1)
	{
		printf("Send..\n");
		can1_loopback();
		HAL_Delay(1000);
        if(is_available(&stdin_uart)) break;
	}
#endif
}
