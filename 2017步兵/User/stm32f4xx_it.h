#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
extern uint8_t Attitude_500Hz ;
extern uint8_t Motor_100Hz ;
extern uint8_t Com_10Hz ;
extern uint8_t Set_2Hz ;
extern uint8_t SystemInitReady;
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
