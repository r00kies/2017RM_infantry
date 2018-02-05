#ifndef __CAN2_H__
#define __CAN2_H__

#include <stm32f4xx.h>
#include "laser.h"
#include "can1.h"

#define pitch_max 15.0
#define yaw_max 720.0				//cyq:ÔÆÌ¨½Ç¶ÈµÄ·¶Î§
 typedef  struct    
 {        
     u16 centric_angle[6];
     u16 real_centric_angle[6];
	 u16 real_current[6];
	 u16 target_current[6];
	 u8 hall_switch[6];
	 
 }st_Gimbal;
extern st_Gimbal  Gimbal_DATA;
extern u32 first_yaw_angle,first_pitch_angle;
void CAN2_Configuration(void);
void GYRO_RST(void);
void Encoder_sent(float encoder_angle);
void RM6025Cmd(s16 c205,s16 c206);
void CAN2_TX_IRQHandler(void);
#endif 
