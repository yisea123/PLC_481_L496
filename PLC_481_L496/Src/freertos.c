/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;
osThreadId myTask07Handle;
osThreadId myTask08Handle;
osThreadId myTask09Handle;
osThreadId myTask10Handle;
osThreadId myTask11Handle;
osThreadId myTask12Handle;
osThreadId myTask13Handle;
osThreadId myTask14Handle;
osThreadId myTask15Handle;
osThreadId myTask16Handle;
osThreadId myTask17Handle;
osThreadId myTask18Handle;
osThreadId myTask19Handle;
osThreadId myTask20Handle;
osThreadId myTask21Handle;
osThreadId myTask22Handle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void Acceleration_Task(void const * argument);
void Velocity_Task(void const * argument);
void Displacement_Task(void const * argument);
void Q_Average_A(void const * argument);
void Q_Average_V(void const * argument);
void Q_Average_D(void const * argument);
void ADC_supply_voltage(void const * argument);
void Lights_Task(void const * argument);
void DAC_Task(void const * argument);
void Display_Task(void const * argument);
void Button_Task(void const * argument);
void Modbus_Receive_Task(void const * argument);
void Modbus_Transmit_Task(void const * argument);
void Master_Modbus_Receive(void const * argument);
void Master_Modbus_Transmit(void const * argument);
void Data_Storage_Task(void const * argument);
void TriggerLogic_Task(void const * argument);
void Relay_1_Task(void const * argument);
void Relay_2_Task(void const * argument);
void HART_Receive_Task(void const * argument);
void HART_Transmit_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, Acceleration_Task, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, Velocity_Task, osPriorityIdle, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, Displacement_Task, osPriorityIdle, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, Q_Average_A, osPriorityIdle, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, Q_Average_V, osPriorityIdle, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* definition and creation of myTask07 */
  osThreadDef(myTask07, Q_Average_D, osPriorityIdle, 0, 128);
  myTask07Handle = osThreadCreate(osThread(myTask07), NULL);

  /* definition and creation of myTask08 */
  osThreadDef(myTask08, ADC_supply_voltage, osPriorityIdle, 0, 128);
  myTask08Handle = osThreadCreate(osThread(myTask08), NULL);

  /* definition and creation of myTask09 */
  osThreadDef(myTask09, Lights_Task, osPriorityIdle, 0, 128);
  myTask09Handle = osThreadCreate(osThread(myTask09), NULL);

  /* definition and creation of myTask10 */
  osThreadDef(myTask10, DAC_Task, osPriorityIdle, 0, 128);
  myTask10Handle = osThreadCreate(osThread(myTask10), NULL);

  /* definition and creation of myTask11 */
  osThreadDef(myTask11, Display_Task, osPriorityIdle, 0, 128);
  myTask11Handle = osThreadCreate(osThread(myTask11), NULL);

  /* definition and creation of myTask12 */
  osThreadDef(myTask12, Button_Task, osPriorityIdle, 0, 128);
  myTask12Handle = osThreadCreate(osThread(myTask12), NULL);

  /* definition and creation of myTask13 */
  osThreadDef(myTask13, Modbus_Receive_Task, osPriorityIdle, 0, 128);
  myTask13Handle = osThreadCreate(osThread(myTask13), NULL);

  /* definition and creation of myTask14 */
  osThreadDef(myTask14, Modbus_Transmit_Task, osPriorityIdle, 0, 128);
  myTask14Handle = osThreadCreate(osThread(myTask14), NULL);

  /* definition and creation of myTask15 */
  osThreadDef(myTask15, Master_Modbus_Receive, osPriorityIdle, 0, 128);
  myTask15Handle = osThreadCreate(osThread(myTask15), NULL);

  /* definition and creation of myTask16 */
  osThreadDef(myTask16, Master_Modbus_Transmit, osPriorityIdle, 0, 128);
  myTask16Handle = osThreadCreate(osThread(myTask16), NULL);

  /* definition and creation of myTask17 */
  osThreadDef(myTask17, Data_Storage_Task, osPriorityIdle, 0, 128);
  myTask17Handle = osThreadCreate(osThread(myTask17), NULL);

  /* definition and creation of myTask18 */
  osThreadDef(myTask18, TriggerLogic_Task, osPriorityIdle, 0, 128);
  myTask18Handle = osThreadCreate(osThread(myTask18), NULL);

  /* definition and creation of myTask19 */
  osThreadDef(myTask19, Relay_1_Task, osPriorityIdle, 0, 128);
  myTask19Handle = osThreadCreate(osThread(myTask19), NULL);

  /* definition and creation of myTask20 */
  osThreadDef(myTask20, Relay_2_Task, osPriorityIdle, 0, 128);
  myTask20Handle = osThreadCreate(osThread(myTask20), NULL);

  /* definition and creation of myTask21 */
  osThreadDef(myTask21, HART_Receive_Task, osPriorityIdle, 0, 128);
  myTask21Handle = osThreadCreate(osThread(myTask21), NULL);

  /* definition and creation of myTask22 */
  osThreadDef(myTask22, HART_Transmit_Task, osPriorityIdle, 0, 128);
  myTask22Handle = osThreadCreate(osThread(myTask22), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		osDelay(500);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    osDelay(500);
		
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Acceleration_Task function */
void Acceleration_Task(void const * argument)
{
  /* USER CODE BEGIN Acceleration_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Acceleration_Task */
}

/* Velocity_Task function */
void Velocity_Task(void const * argument)
{
  /* USER CODE BEGIN Velocity_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Velocity_Task */
}

/* Displacement_Task function */
void Displacement_Task(void const * argument)
{
  /* USER CODE BEGIN Displacement_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Displacement_Task */
}

/* Q_Average_A function */
void Q_Average_A(void const * argument)
{
  /* USER CODE BEGIN Q_Average_A */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Q_Average_A */
}

/* Q_Average_V function */
void Q_Average_V(void const * argument)
{
  /* USER CODE BEGIN Q_Average_V */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Q_Average_V */
}

/* Q_Average_D function */
void Q_Average_D(void const * argument)
{
  /* USER CODE BEGIN Q_Average_D */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Q_Average_D */
}

/* ADC_supply_voltage function */
void ADC_supply_voltage(void const * argument)
{
  /* USER CODE BEGIN ADC_supply_voltage */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ADC_supply_voltage */
}

/* Lights_Task function */
void Lights_Task(void const * argument)
{
  /* USER CODE BEGIN Lights_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Lights_Task */
}

/* DAC_Task function */
void DAC_Task(void const * argument)
{
  /* USER CODE BEGIN DAC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DAC_Task */
}

/* Display_Task function */
void Display_Task(void const * argument)
{
  /* USER CODE BEGIN Display_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Display_Task */
}

/* Button_Task function */
void Button_Task(void const * argument)
{
  /* USER CODE BEGIN Button_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Button_Task */
}

/* Modbus_Receive_Task function */
void Modbus_Receive_Task(void const * argument)
{
  /* USER CODE BEGIN Modbus_Receive_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Modbus_Receive_Task */
}

/* Modbus_Transmit_Task function */
void Modbus_Transmit_Task(void const * argument)
{
  /* USER CODE BEGIN Modbus_Transmit_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Modbus_Transmit_Task */
}

/* Master_Modbus_Receive function */
void Master_Modbus_Receive(void const * argument)
{
  /* USER CODE BEGIN Master_Modbus_Receive */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Master_Modbus_Receive */
}

/* Master_Modbus_Transmit function */
void Master_Modbus_Transmit(void const * argument)
{
  /* USER CODE BEGIN Master_Modbus_Transmit */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Master_Modbus_Transmit */
}

/* Data_Storage_Task function */
void Data_Storage_Task(void const * argument)
{
  /* USER CODE BEGIN Data_Storage_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Data_Storage_Task */
}

/* TriggerLogic_Task function */
void TriggerLogic_Task(void const * argument)
{
  /* USER CODE BEGIN TriggerLogic_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TriggerLogic_Task */
}

/* Relay_1_Task function */
void Relay_1_Task(void const * argument)
{
  /* USER CODE BEGIN Relay_1_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Relay_1_Task */
}

/* Relay_2_Task function */
void Relay_2_Task(void const * argument)
{
  /* USER CODE BEGIN Relay_2_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Relay_2_Task */
}

/* HART_Receive_Task function */
void HART_Receive_Task(void const * argument)
{
  /* USER CODE BEGIN HART_Receive_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HART_Receive_Task */
}

/* HART_Transmit_Task function */
void HART_Transmit_Task(void const * argument)
{
  /* USER CODE BEGIN HART_Transmit_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HART_Transmit_Task */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
