#include "task_can.h"

static unsigned char can_tx_success_flag = 0;
extern u8 Gear;
Chassis RM3510_DATA;
s16 current_out[6];
s16 PITCH_speed;
//can通信任务
void can_task(void *pdata)
{
	while(1)
	{
		ALL_can_tx();
		delay_ms(5);
	}
}

void ALL_can_tx()
{
      if(RC_Ctl.rc.s2==2||RC_Ctl.rc.s2==3)
			 {
                  RM3510_control();
				          Gimban_ctrl();
                  
               // Gimban_Camera_ctrl(); 
			 }
			 else
			 {
                RM_Driver_Cmd(CAN1,0,0);

				        RM3510_Cmd(CAN1,0,0,0,0);

			          LED_RED_OFF();
			 } 

}

/*************************************************************************
                          CAN1_TX_IRQHandler
描述：CAN1的发送中断函数
*************************************************************************/
void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
       can_tx_success_flag=1;
    }
}
/*************************************************************************
                          CAN1_RX0_IRQHandler
描述：CAN1的接收中断函数
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg canRxMsg;
    
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
      
         CAN_Receive(CAN1, CAN_FIFO0, &canRxMsg);
		    //底盘电流采集板反馈电流值
        if(canRxMsg.StdId==0x300)
        {             
                current_out[1]=(canRxMsg.Data[0]<<8)|canRxMsg.Data[1];
                current_out[2]=(canRxMsg.Data[2]<<8)|canRxMsg.Data[3];
                current_out[3]=(canRxMsg.Data[4]<<8)|canRxMsg.Data[5];
					      current_out[4]=(canRxMsg.Data[6]<<8)|canRxMsg.Data[7];
				}
        switch(canRxMsg.StdId)
         {
             case 0x201:
                                
                 RM3510_DATA.speed[1]=((u16)canRxMsg.Data[2]<<8)|canRxMsg.Data[3];
                 break;
             case 0x202:
                               
                 RM3510_DATA.speed[2]=((u16)canRxMsg.Data[2]<<8)|canRxMsg.Data[3];
						     break;
             case 0x203:
                               
                 RM3510_DATA.speed[3]=((u16)canRxMsg.Data[2]<<8)|canRxMsg.Data[3];
						     break;
             case 0x204:
                 
     						 RM3510_DATA.speed[4]=((u16)canRxMsg.Data[2]<<8)|canRxMsg.Data[3];
						     break;
						  case 0x205://yaw
                 Gimbal_DATA.real_centric_angle[4]=canRxMsg.Data[0]*256+canRxMsg.Data[1];
                 Gimbal_DATA.centric_angle[4]=transe_M_angle(Gimbal_DATA.real_centric_angle[4], first_yaw_angle);
                 Gimbal_DATA.real_current[4]=((u16)canRxMsg.Data[2]<<8)|canRxMsg.Data[3];
                 Gimbal_DATA.target_current[4]=((u16)canRxMsg.Data[4]<<8)|canRxMsg.Data[5];
                 Gimbal_DATA.hall_switch[4]=canRxMsg.Data[6];
						     break;
             case 0x206://pitch
                 Gimbal_DATA.real_centric_angle[5]=canRxMsg.Data[0]*256+canRxMsg.Data[1];
                 Gimbal_DATA.centric_angle[5] =transe_M_angle(Gimbal_DATA.real_centric_angle[5],first_pitch_angle);
                 Gimbal_DATA.real_current[5]  =((u16)canRxMsg.Data[2]<<8)|canRxMsg.Data[3];
                 Gimbal_DATA.target_current[5]=((u16)canRxMsg.Data[4]<<8)|canRxMsg.Data[5];
                 Gimbal_DATA.hall_switch[5]   = canRxMsg.Data[6];
						     break;
							   
             default:break;
         }
    }
}


//
 void RM_Driver_Cmd(CAN_TypeDef *CANx,s16 c205,s16 c206)
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
     CAN_Transmit(CANx,&canTxMsg);
	 while(can_tx_success_flag == 0);
 }
  //RM3510 CAN 数据发送
 void RM3510_Cmd(CAN_TypeDef *CANx,s16 c201,s16 c202,s16 c203,s16 c204)
  {
     CanTxMsg canTxMsg;
     canTxMsg.StdId=0x200;
     canTxMsg.IDE=CAN_ID_STD;
     canTxMsg.RTR=CAN_RTR_DATA;
     canTxMsg.DLC=8;
     canTxMsg.Data[0]=h8b(c201);
     canTxMsg.Data[1]=l8b(c201);
     canTxMsg.Data[2]=h8b(c202);
     canTxMsg.Data[3]=l8b(c202);
     canTxMsg.Data[4]=h8b(c203);
     canTxMsg.Data[5]=l8b(c203);
     canTxMsg.Data[6]=h8b(c204);
     canTxMsg.Data[7]=l8b(c204);
	   can_tx_success_flag=0;
     CAN_Transmit(CANx,&canTxMsg);
	  while(can_tx_success_flag == 0);
 }
	 