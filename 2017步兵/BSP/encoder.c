
#include "encoder.h"

void Encoder_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ICInitTypeDef  TIM2_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM2时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA0

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2); //PA0复用位定时器5
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=0;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=0xffff;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
    
	 TIM_ITRxExternalClockConfig(TIM2,TIM_TS_ETRF);
	TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF,TIM_ExtTRGPolarity_NonInverted, 0);	
	
	
  TIM_Cmd(TIM2,ENABLE ); 	//使能定时器5
    
}

u32 Encoder_Get_CNT(void)
{
    u32 cnt = 0;
    cnt = TIM2->CNT;
    TIM2->CNT=0;
    return cnt;
}
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
	{
	   TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
  } 
}