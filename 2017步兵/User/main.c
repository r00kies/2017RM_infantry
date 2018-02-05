#include "main.h"
#include "includes.h"
#include "task_door.h"
#include "TASK_key_board.h"
//开始任务
#define START_TASK_PRIO      	  11
#define START_STK_SIZE  				64
OS_STK START_TASK_STK[START_STK_SIZE];
//can任务
#define CAN_TASK_PRIO       		5
#define CAN_STK_SIZE  		    		256
OS_STK CAN_TASK_STK[CAN_STK_SIZE];
//控制任务
#define CONTROL_TASK_PRIO       			6
#define CONTROL_STK_SIZE  		    		512
OS_STK CONTROL_TASK_STK[CONTROL_STK_SIZE];
//调试任务
#define DBUG_TASK_PRIO       	   10
#define DBUG_STK_SIZE  		    		256
OS_STK DBUG_TASK_STK[DBUG_STK_SIZE];
//key_scan任务
#define KEY_TASK_PRIO       	   9
#define KEY_STK_SIZE  		    		256
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
//舱门控制任务
#define DOOR_TASK_PRIO       		  7
#define DOOR_STK_SIZE  					  64
OS_STK DOOR_TASK_STK[DOOR_STK_SIZE];
//闪灯任务
#define LED_TASK_PRIO       			8                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
#define LED_STK_SIZE  		    		64
OS_STK LED_TASK_STK[LED_STK_SIZE];
//初始化
int main(void)
{
 
    int i = 0;
    BSP_Init();  
    delayms(500);//delay 500ms， 等待mpu6050上电稳定     
    while(MPU6050_Initialization() == 0xff) 
    {
        i++;     //如果一次初始化没有成功，那就再来一次                     
        if(i>10) //如果初始化一直不成功，那就没希望了，进入死循环，蜂鸣器一直叫
        {
            while(1) 
            {
                LED_RED_TOGGLE();
                delayms(500);
                
            }
        }  
    }
		    pid_init_struct();
        MPU6050_Interrupt_Configuration();
		    TIM6_Start();
        Data_Read();
        iwdg_init();
        delay_init(168);
		    OSInit();  	 				//初始化UCOSII
  	    OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	      OSStart();
		    
}
 //开始任务
  void start_task(void *pdata)
   {  
		    OS_CPU_SR cpu_sr=0;
		    pdata = pdata;
        OSStatInit();							 
				OS_ENTER_CRITICAL();			   
				OSTaskCreate(can_task,(void *)0,(OS_STK*)&CAN_TASK_STK[CAN_STK_SIZE-1],CAN_TASK_PRIO);						   
				OSTaskCreate(control_task,(void *)0,(OS_STK*)&CONTROL_TASK_STK[CONTROL_STK_SIZE-1],CONTROL_TASK_PRIO);	 				   
				OSTaskCreate(dbug_task,(void *)0,(OS_STK*)&DBUG_TASK_STK[DBUG_STK_SIZE-1],DBUG_TASK_PRIO);
        OSTaskCreate(key_board_task,(void *)0,(OS_STK*)&KEY_TASK_STK[KEY_STK_SIZE-1],KEY_TASK_PRIO);		 
				OSTaskCreate(door_task,(void *)0,(OS_STK*)&DOOR_TASK_STK[DOOR_STK_SIZE-1],DOOR_TASK_PRIO);	 				   
				OSTaskCreate(led_task,(void *)0,(OS_STK*)&LED_TASK_STK[LED_STK_SIZE-1],LED_TASK_PRIO);	 				   	 				   
				OSTaskSuspend(START_TASK_PRIO);	
				OS_EXIT_CRITICAL();				
   }
