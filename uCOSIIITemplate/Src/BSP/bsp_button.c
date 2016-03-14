/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <bsp.h>
#include <stm32f4xx_hal.h>
#include <os.h>

extern OS_SEM AppTaskSemLed1;
extern OS_TCB AppTaskServerTCB;

extern void EXTI0_IRQHandler(void);

void BSP_IntHandlerButtonBlue(void)
{
	OS_ERR err;
	
  EXTI0_IRQHandler();
  OSTaskSemPost(&AppTaskServerTCB,
								OS_OPT_POST_NONE,
								&err);
}

CPU_INT32U BSP_ButtonBlueRd(void)
{
	return HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
}
