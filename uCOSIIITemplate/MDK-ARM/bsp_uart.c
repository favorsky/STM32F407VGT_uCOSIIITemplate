#include <bsp.h>
#include <stm32f4xx_hal.h>
#include <os.h>

extern void USART3_IRQHandler(void);

void BSP_IntHandlerUSARTSec(void)
{
	USART3_IRQHandler();
}

void BSP_USART_Putc(UART_HandleTypeDef* USARTx, uint8_t* str)
{
	while (*str) {
		HAL_UART_Transmit(USARTx,str++,1,20000);
	}
}
