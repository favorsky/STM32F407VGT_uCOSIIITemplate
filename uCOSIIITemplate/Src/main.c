/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include  <stdarg.h>
#include  <stdio.h>
#include  <math.h>

#include  <cpu.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <os.h>
#include  <os_app_hooks.h>

#include  <app_cfg.h>
#include  <bsp.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* Private variables ---------------------------------------------------------*/
static  OS_TCB       AppTaskStartTCB;
static  CPU_STK      AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

        OS_TCB       AppTaskLed1TCB;
static  CPU_STK      AppTaskLed1Stk[APP_CFG_TASK_LED1_STK_SIZE];

static  OS_TCB       AppTaskUARTTCB;
static  CPU_STK      AppTaskUARTStk[APP_CFG_TASK_UART_STK_SIZE];

        OS_TCB       AppTaskServerTCB;
static  CPU_STK      AppTaskServerStk[APP_CFG_TASK_SERVER_STK_SIZE];

OS_SEM       AppTaskSemLed1; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern  CPU_INT32U  BSP_ButtonBlueRd(void);
extern  void        BSP_USART_Putc(UART_HandleTypeDef* UARTx, uint8_t* str);



static  void  AppTaskStart (void  *p_arg);
static  void  AppTaskCreate(void);
static  void  AppObjCreate (void);

static  void  AppTaskLed1(void);
static  void  AppTaskUARTSend(void * p_arg);
static  void  AppTaskServer(void *p_arg);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
    OS_ERR   err;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  Mem_Init();  
	Math_Init();  
	#if (CPU_CFG_NAME_EN == DEF_ENABLED)
    CPU_NameSet((CPU_CHAR *)"STM32F407VG",
                (CPU_ERR  *)&err);
  #endif

    BSP_IntDisAll();                                            /* Disable all Interrupts.                              */
     

		 
    OSInit(&err);                                               /* Init uC/OS-III.                                      */
    App_OS_SetAllHooks();
    OSTaskCreate(&AppTaskStartTCB,                              /* Create the start task                                */
                  "App Task Start",
                  AppTaskStart,
                  0u,
                  APP_CFG_TASK_START_PRIO,
                 &AppTaskStartStk[0u],
                  AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE / 10u],
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &err);

    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTaskStart (void *p_arg)
{
    OS_ERR      err;


    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */

#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif

#if (APP_CFG_SERIAL_EN == DEF_ENABLED)

#endif
    AppTaskCreate();                                            /* Create Application tasks                             */
    AppObjCreate();
		
		OSTaskDel((OS_TCB * )0,&err);

}

/*
*********************************************************************************************************
*                                          AppTaskCreate()
*
* Description : Create Application Tasks.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppTaskCreate (void)
{
    OS_ERR  os_err;

                                                                /* ---------- CREATE KERNEL OBJECTS TEST TASK --------- */
	  OSTaskCreate(&AppTaskLed1TCB,                               /* Create Led1 task                                */
                  "App Task Led1",
                  (OS_TASK_PTR)AppTaskLed1,
                  0u,
                  APP_CFG_TASK_LED1_PRIO,
                 &AppTaskLed1Stk[0u],
                  AppTaskLed1Stk[APP_CFG_TASK_LED1_STK_SIZE / 10u],
                  APP_CFG_TASK_LED1_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
	
								 
		OSTaskCreate(&AppTaskUARTTCB,
								 "App Task UART",
								 (OS_TASK_PTR)AppTaskUARTSend,
								 0u,
								 APP_CFG_TASK_UART_PRIO,
								 &AppTaskUARTStk[0u],
								 AppTaskUARTStk[APP_CFG_TASK_UART_STK_SIZE / 10u],
								 APP_CFG_TASK_UART_STK_SIZE,
								 0u,
								 0u,
								 0u,
								 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
								 &os_err);
								 
    OSTaskCreate(&AppTaskServerTCB,
								"App Task Server",
								(OS_TASK_PTR)AppTaskServer,
								0u,
								APP_CFG_TASK_SERVER_PRIO,
								&AppTaskServerStk[0u],
								AppTaskServerStk[APP_CFG_TASK_SERVER_STK_SIZE / 10u],
								APP_CFG_TASK_SERVER_STK_SIZE,
								0u,
								0u,
								0u,
								(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
								&os_err);
}


/*
*********************************************************************************************************
*                                          AppObjCreate()
*
* Description : Create Application Kernel Objects.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/
static  void  AppObjCreate (void)
{
    OS_ERR  os_err;
	
	  OSSemCreate(&AppTaskSemLed1,
							  "Start blink",
								0,
							  &os_err);
}


/*
*********************************************************************************************************
*                                          AppTaskLed1()
*
* Description : Test uC/OS-III objects.
*
* Argument(s) : p_arg is the argument passed to 'AppTaskObj0' by 'OSTaskCreate()'.
*
* Return(s)   : none
*
* Caller(s)   : This is a task
*
* Note(s)     : none.
*********************************************************************************************************
*/
static void AppTaskLed1(void)
{
	OS_ERR os_err;
	while(DEF_TRUE)
	{
		BSP_LED_Toggle(1u);
		OSTimeDlyHMSM(0u, 0u, 0u, 50u,
                  OS_OPT_TIME_HMSM_STRICT,
                  &os_err);
	}
}



/*
*********************************************************************************************************
*                                             AppTaskUartSend()
*
* Description : This task transmit message by usart.
*
* Argument(s) : p_arg   is the argument 
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static void AppTaskUARTSend(void * p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
  CPU_INT08U tx[] = "abcde";
	while(DEF_TRUE)
	{
	
	BSP_USART_Putc(&huart3,tx);
		
	OSTimeDlyHMSM(0u,0u,0u,500u,
								OS_OPT_TIME_HMSM_STRICT,
								&err);
	}
	
	
}

static void AppTaskServer(void * p_arg)
{
	OS_ERR err;
	CPU_TS ts;
	
	while(DEF_TRUE)
	{
		OSTaskSemPend(0u,
									OS_OPT_PEND_BLOCKING,
									&ts,
									&err);
		OSTimeDlyHMSM(0u,0u,0u,80u,
									OS_OPT_TIME_HMSM_STRICT,
									&err);
		if(BSP_ButtonBlueRd() == 1u)
		{
			OSTaskSemPost(&AppTaskLed1TCB,
										OS_OPT_POST_NONE,
										&err);
		}
	}
}


/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
