#include "can2.h"
#include "led.h"
#include "app.h"
st_Gimbal  Gimbal_DATA;
u32 first_yaw_angle,first_pitch_angle;   
/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/

void CAN2_Configuration(void)
{
CAN_InitTypeDef        can;
     CAN_FilterInitTypeDef  can_filter;
     GPIO_InitTypeDef       gpio;
     NVIC_InitTypeDef       nvic;

     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

     GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
     GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); 

     gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12 ;
     gpio.GPIO_Mode = GPIO_Mode_AF;
     GPIO_Init(GPIOB, &gpio);

     nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
     nvic.NVIC_IRQChannelPreemptionPriority = 0;
     nvic.NVIC_IRQChannelSubPriority = 1;
     nvic.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&nvic);
     
		     nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);  
		
     CAN_DeInit(CAN2);
     CAN_StructInit(&can);

     can.CAN_TTCM = DISABLE;
     can.CAN_ABOM = DISABLE;    
     can.CAN_AWUM = DISABLE;    
     can.CAN_NART = DISABLE;    
     can.CAN_RFLM = DISABLE;    
     can.CAN_TXFP = ENABLE;     
     can.CAN_Mode = CAN_Mode_Normal; 
     can.CAN_SJW  = CAN_SJW_1tq;
     can.CAN_BS1 = CAN_BS1_9tq;
     can.CAN_BS2 = CAN_BS2_4tq;
     can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
     CAN_Init(CAN2, &can);
     
     can_filter.CAN_FilterNumber=14;
     can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
     can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
     can_filter.CAN_FilterIdHigh=0x0000;
     can_filter.CAN_FilterIdLow=0x0000;
     can_filter.CAN_FilterMaskIdHigh=0x0000;
     can_filter.CAN_FilterMaskIdLow=0x0000;
     can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
     can_filter.CAN_FilterActivation=ENABLE;
     CAN_FilterInit(&can_filter);
     
     CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
		 CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}
static unsigned char can_tx_success_flag = 0;
void CAN2_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
       can_tx_success_flag=1;
    }
}

void CAN2_RX0_IRQHandler(void)
 {
     CanRxMsg canRxMsg;
     if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
     {
         
         CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
         CAN_Receive(CAN2, CAN_FIFO0, &canRxMsg);
         switch(canRxMsg.StdId)
         {
//             case 0x201:

//             case 0x202:

//             case 0x203:

//             case 0x204:

             case 0x205:
                 Gimbal_DATA.real_centric_angle[4]=canRxMsg.Data[0]*256+canRxMsg.Data[1];
                 Gimbal_DATA.centric_angle[4]=transe_M_angle(Gimbal_DATA.real_centric_angle[4],first_yaw_angle);
                 Gimbal_DATA.real_current[4]=((u16)canRxMsg.Data[2]<<8)|canRxMsg.Data[3];
                 Gimbal_DATA.target_current[4]=((u16)canRxMsg.Data[4]<<8)|canRxMsg.Data[5];
                 Gimbal_DATA.hall_switch[4]=canRxMsg.Data[6];
						 break;
             case 0x206:
                 Gimbal_DATA.real_centric_angle[5]=canRxMsg.Data[0]*256+canRxMsg.Data[1];
                 Gimbal_DATA.centric_angle[5] =transe_M_angle(Gimbal_DATA.real_centric_angle[5],first_pitch_angle);
                 Gimbal_DATA.real_current[5]  =((u16)canRxMsg.Data[2]<<8)|canRxMsg.Data[3];
                 Gimbal_DATA.target_current[5]=((u16)canRxMsg.Data[4]<<8)|canRxMsg.Data[5];
                 Gimbal_DATA.hall_switch[5]   = canRxMsg.Data[6];
						  break;
             default:
         }
//				  printf("%d    %d\r\n",Gimbal_DATA.centric_angle[4],Gimbal_DATA.centric_angle[5]);
         
     }
 }

#define h8b(num) ((num) >> 8)
#define l8b(num) ((num)&0xff)
 void RM6025Cmd(s16 c205,s16 c206)//205 :pitch 206 :yaw
 {
     CanTxMsg canTxMsg;
     canTxMsg.StdId=0x1FF;
     canTxMsg.IDE=CAN_ID_STD;
     canTxMsg.RTR=CAN_RTR_DATA;
     canTxMsg.DLC=8;
     canTxMsg.Data[0]=h8b(c205);
     canTxMsg.Data[1]=l8b(c205);
     canTxMsg.Data[2]=h8b(c206);
     canTxMsg.Data[3]=l8b(c206);
     canTxMsg.Data[4]=0;
     canTxMsg.Data[5]=0;
     canTxMsg.Data[6]=0;
     canTxMsg.Data[7]=0;
	   can_tx_success_flag=0;
     CAN_Transmit(CAN2,&canTxMsg);
      
	 while(can_tx_success_flag == 0);
 }