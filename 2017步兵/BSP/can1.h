#ifndef __CAN1_H__
#define __CAN1_H__
#include <stm32f4xx.h>
void CAN1_Configuration(void);
 typedef  struct    
 {        
   s16 angle[6];
	 s16 speed[6];	 
 }Chassis;
 
#endif 
