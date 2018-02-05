#include "main.h"
void iwdg_init()
	{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetReload(0xfff);
	IWDG_SetPrescaler(IWDG_Prescaler_32);
    IWDG_ReloadCounter();
	IWDG_Enable();
	}
