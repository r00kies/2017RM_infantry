#include "bsp.h"

void BSP_Init(void){
    
    /* Configure the NVIC Preemption Priority Bits */
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   
        LED_Configuration();            //指示灯初始化
        Key_Configuration(); 
        Buzzer_Configuration();
	      RELAY_init();                   //初始化继电器
        LASER_Configuration();
        CAN1_Configuration();            //初始化CAN
        CAN2_Configuration(); 
        USART1_Configuration();          //串口1初始化
        USART2_Configuration();
        USART3_Configuration();
        TIM6_Configuration();
        Encoder_Configuration();
		    TIM12_CH12_Init(99,16799);       //500hz 20ms周期
	      PWM_DOOR=3;                      //上电舱门关闭
        PWM1_Configuration();  
        PWM2_Configuration(); 

       //设定占空比为1000，初始化摩擦轮电调
        PWM1 =1000;
        PWM2 =1000;
}


