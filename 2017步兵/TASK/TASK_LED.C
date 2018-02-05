#include "task_led.h"

//LED任务
void led_task(void *pdata)
{
	while(1)
	{
    			static u8 iwdg_flag;
		          Turn_On_LEDs();		            		            
							iwdg_flag++;
							if(iwdg_flag%2)
							 GPIO_ResetBits(GPIOA, GPIO_Pin_6);//闪灯
							else
							 GPIO_SetBits(GPIOA, GPIO_Pin_6);
							if(iwdg_flag>5)//看门狗喂狗
							{
								iwdg_flag=0;
								IWDG_ReloadCounter();
							}
	            delay_ms(10);						
	}									 
}
void Spark_LEDs(void)
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_7 | GPIO_Pin_6);
  delay_ms(200);
  GPIO_SetBits(GPIOA, GPIO_Pin_7 | GPIO_Pin_6);
  delay_ms(200);
  GPIO_ResetBits(GPIOA, GPIO_Pin_7 | GPIO_Pin_6);
  delay_ms(200);
  GPIO_SetBits(GPIOA, GPIO_Pin_7 | GPIO_Pin_6);
  delay_ms(200);
  GPIO_ResetBits(GPIOA, GPIO_Pin_7 | GPIO_Pin_6);
  delay_ms(200);
  GPIO_SetBits(GPIOA, GPIO_Pin_7 | GPIO_Pin_6);
}
void Spark_GreenLED(void)
{
  LED_GREEN_ON();
  delay_ms(200);
  LED_GREEN_OFF();
  delay_ms(200);
}
void Spark_RedLED(void)
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_7);
  delay_ms(200);
  GPIO_SetBits(GPIOA, GPIO_Pin_7);
  delay_ms(200);
  GPIO_ResetBits(GPIOA, GPIO_Pin_7);
  delay_ms(200);
  GPIO_SetBits(GPIOA, GPIO_Pin_7);
  delay_ms(200);
  GPIO_ResetBits(GPIOA, GPIO_Pin_7);
  delay_ms(200);
  GPIO_SetBits(GPIOA, GPIO_Pin_7);
}
void Turn_On_LEDs(void)
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_7 | GPIO_Pin_6);
}
void Turn_Off_LEDs(void)
{
  GPIO_SetBits(GPIOA, GPIO_Pin_7 | GPIO_Pin_6);
}
