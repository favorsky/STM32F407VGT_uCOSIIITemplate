#include <bsp.h>
#include <stm32f4xx_hal.h>
#include <os.h>

extern void USART3_IRQHandler(void);

void BSP_IntHandlerUSARTSec(void)
{
	USART3_IRQHandler();
}

