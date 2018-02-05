#include "pwm.h"
#include "bsp.h"
extern s16 Rammer_Moto_Speed,Rammer_Moto_Measured_Speed;//拨弹电机速度
extern s16 Rammer_moto_position;//拨弹电机位置
extern s16 Rammer_Moto_Speed_out;//拨弹电机位置环输出
extern int count;
//摩擦轮TIM3_CH3,TIM3_CH4
void PWM1_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB,&gpio);
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource0, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource1, GPIO_AF_TIM3);    
    
    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 2500;   //2.5ms   
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3,&tim);
    
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC3Init(TIM3,&oc);
    TIM_OC4Init(TIM3,&oc);
    
    TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM3,ENABLE);

    TIM_Cmd(TIM3,ENABLE);
   
}
//舵机
//TIM12_CH1 PB14
//TIM12_CH2 PB15
void TIM12_CH12_Init(u16 arr,u16 psc)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB,&gpio);
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource14, GPIO_AF_TIM12);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource15, GPIO_AF_TIM12);    
    
    tim.TIM_Prescaler = psc;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = arr;   
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM12,&tim);
    
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM12,&oc);
    TIM_OC2Init(TIM12,&oc);
    
    TIM_OC1PreloadConfig(TIM12,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM12,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM12,ENABLE);

    TIM_Cmd(TIM12,ENABLE);
   
}


/*-H1--(PB8--TIM4_CH3)--*/
/*-H2--(PB9--TIM4_CH4)--*/

//拨弹电机PWM
void PWM2_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
   
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB,&gpio);
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);
       
    tim.TIM_Prescaler = 8-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1000;   //21 KHz
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4,&tim);
    
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;

    TIM_OC3Init(TIM4,&oc);
    TIM_OC4Init(TIM4,&oc);

    TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);    
    TIM_ARRPreloadConfig(TIM4,ENABLE);
    TIM_CtrlPWMOutputs(TIM4,ENABLE);
    TIM_Cmd(TIM4,ENABLE);
}

static void SetMotorVoltage(float f_tVal)
{

	if(f_tVal>0)
	{
		if(count%2)
		{
      PWM3=f_tVal;
	    PWM4=0;
		}
		else
		{
			PWM3=f_tVal;
	    PWM4=1000;
		}
	}
	else
	{
      PWM3=0;
      PWM4=0;
	}
}
/******************************************************************************
拨弹电机输出限幅
 ******************************************************************************/

static void MotorSpeedOut(float f_Val) 
{

 if(f_Val > 0) 
 f_Val += 10;
 else if(f_Val < 0) 
 f_Val -= 10;
 if(f_Val>1000)
 {
 f_Val=1000;
 }
 else if(f_Val<-1000)
 {
 f_Val=-1000;
 }
 SetMotorVoltage(f_Val);
}
void Rammer_moto_run_speed(void)
{
    float Pid_Out;
    Rammer_Moto_Measured_Speed=Encoder_Get_CNT();
    Pid_Out=PID_Calculate(&PID_Rammer_moto,Rammer_Moto_Measured_Speed,Rammer_Moto_Speed);
    MotorSpeedOut(Pid_Out);
}
void Rammer_run_position(s16 expect_position)//主控板原因不能用
{
	  float Pid_Out;
	  Rammer_Moto_Measured_Speed=Encoder_Get_CNT();
	  Rammer_moto_position+=Rammer_Moto_Measured_Speed;
	  Rammer_Moto_Speed_out=PID_Calculate(&PID_Rammer_moto,Rammer_moto_position,expect_position);
	  Pid_Out=PID_Calculate(&PID_Rammer_moto,Rammer_Moto_Measured_Speed,Rammer_Moto_Speed_out);
    MotorSpeedOut(Pid_Out);
}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	