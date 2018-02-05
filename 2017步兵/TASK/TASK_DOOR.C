#include "task_door.h"




//舱门控制任务
void door_task(void *pdata)
{
	while(1)
	{
		DOOR_control();
    Turn_Off_LEDs();
		delay_ms(10);
	}									 
}
//舱门控制
//单刀双掷型
void DOOR_control()
{

					  
         if((RC_Ctl.rc.s1==2&&key_jump_e==1)||(RC_Ctl.rc.s1==3&&key_jump_e==0))//打开   
        {
					 PWM_DOOR=8;		//舵机DOOR开		 
					 LED_RED_ON();
	      }
				 else

         {
					 PWM_DOOR=3;//舵机DOOR闭
					 LED_RED_OFF();
         }

 }