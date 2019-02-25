
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern uint16_t raw_adc_value[ADC_BUFFER_SIZE];

volatile uint64_t cpu_load2 = 0;
volatile uint32_t count_idle;
volatile uint32_t value = 0;
volatile uint32_t freeHeapSize = 0;
volatile uint32_t timer_100ms = 0;

extern volatile uint64_t xTimeBefore, xTotalTimeSuspended;

volatile uint32_t temp1 = 0;
volatile uint32_t temp2 = 0;
volatile uint32_t temp3 = 0;
volatile uint32_t temp4 = 0;

extern uint8_t queue_count;
extern float32_t qrms;
extern float32_t qrms_array[8];	

extern xQueueHandle queue;

extern xSemaphoreHandle Semaphore1, Semaphore2, Semaphore3, Semaphore4;

uint8_t status_flash_reg = 0;
extern uint16_t settings[REG_COUNT];
extern uint16_t default_settings[REG_COUNT];

extern DMA_HandleTypeDef hdma_adc1;

void convert_float_and_swap(float32_t float_in, uint16_t* int_out);
void convert_double_and_swap(float64_t double_in, uint16_t* int_out);

extern float32_t break_level_icp;
extern uint16_t slave_adr_mb_master;
extern uint16_t slave_func_mb_master;
extern uint16_t slave_reg_mb_master;
extern uint16_t mb_master_timeout;
extern uint16_t quantity_reg_mb_master;
extern uint16_t coef_A_mb_master;

extern float32_t lo_warning_icp;
extern float32_t hi_warning_icp;
extern float32_t lo_emerg_icp;
extern float32_t hi_emerg_icp;
extern float32_t icp_coef_K;
extern float32_t icp_coef_B;
extern float32_t range_icp;
extern uint8_t filter_mode_icp;


extern float32_t lo_warning_420;
extern float32_t hi_warning_420;
extern float32_t lo_emerg_420;
extern float32_t hi_emerg_420;
extern float32_t coef_ampl_420;
extern float32_t coef_offset_420;
extern float32_t up_user_range_4_20;
extern float32_t down_user_range_4_20;
extern float32_t out_4_20_coef_K;	
extern float32_t out_4_20_coef_B;	

extern float32_t lo_warning_485;
extern float32_t hi_warning_485;
extern float32_t lo_emerg_485;
extern float32_t hi_emerg_485;

extern uint16_t mode_relay;
extern uint8_t source_signal_relay;
extern uint16_t delay_relay;
extern uint16_t delay_relay_exit;

extern uint16_t slave_adr;
extern uint16_t warming_up;
extern float32_t power_supply_warning_lo;
extern float32_t power_supply_warning_hi;

extern float32_t baud_rate_uart_1;
extern float32_t baud_rate_uart_2;
extern float32_t baud_rate_uart_3;
uint32_t boot_timer_counter;	
extern uint8_t bootloader_state;
extern uint8_t receiveBuffer[16];

extern uint16_t channel_ICP_ON;
extern uint16_t channel_4_20_ON;
extern uint16_t channel_485_ON;

volatile uint16_t timer_485_counter = 0;
extern uint8_t break_sensor_485;

extern uint8_t hart_switch_on;
extern uint16_t hart_slave_address;
extern uint16_t hart_slave_numreg;
extern uint8_t hart_func;
extern uint16_t hart_regs_qty;
extern uint16_t hart_timeout_transmit;
extern uint16_t hart_time_poll;

extern uint16_t icp_menu_points_for_showing;
extern uint16_t menu_485_points_for_showing;

extern uint16_t hysteresis_TOC;
extern uint16_t impulse_sign;

extern uint8_t flag_delay_relay_1_4_20;
extern uint8_t relay_permission_1_4_20;
extern uint8_t flag_delay_relay_2_4_20;
extern uint8_t relay_permission_2_4_20;
extern uint16_t timer_delay_relay_1_4_20;
extern uint16_t timer_delay_relay_2_4_20;

extern uint8_t flag_delay_relay_1_icp;
extern uint8_t relay_permission_1_icp;
extern uint16_t timer_delay_relay_1_icp;
extern uint8_t flag_delay_relay_2_icp;
extern uint8_t relay_permission_2_icp;
extern uint16_t timer_delay_relay_2_icp;

extern struct mb_master_delay_relay master_delay_relay_array[REG_485_QTY];

extern uint8_t quit_relay_button;
uint8_t quit_timer = 0;

extern uint8_t adc_bunch; 
extern uint16_t bunch_count_1;		
extern uint16_t bunch_count_2;		

extern xSemaphoreHandle Semaphore_Acceleration;

extern uint8_t worker_status;

extern uint16_t size_moving_average_ZSK;
extern uint64_t trigger_485_ZSK; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	__disable_irq();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
	//Читаем параметры и уставки из flash
	read_init_settings();
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_SPI3_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */



	//Инициализация фильтров
	FilterInit();

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &raw_adc_value, RAW_ADC_BUFFER_SIZE);
	//__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT); /* Disable the half transfer interrupt */
	
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);	
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
	
	//Проверка частоты тактирования (на PA8)
	//HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

//	//Set RTS (for HART Rx)
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	
	/* Enable the UART Transmit Complete Interrupt */ 
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);	
	
	
		
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_HSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 4;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_4);

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

void convert_float_and_swap(float32_t float_in, uint16_t* int_out)
{	
	uint16_t temp = 0;
	
	memcpy(int_out, &float_in, sizeof(float32_t));

	temp = int_out[0];
	int_out[0] = int_out[1];	
	int_out[1] = temp;		
}	


void convert_to_swFloat(float32_t float_in, uint16_t* int_out)
{	
	uint16_t temp = 0;
	
	memcpy(int_out, &float_in, sizeof(float32_t));

	//temp = int_out[0];
	
//	int_out[0] = temp;	
//	int_out[1] = int_out[1];
}	

void convert_double_and_swap(float64_t double_in, uint16_t* int_out)
{	
	uint16_t temp1 = 0;
	uint16_t temp2 = 0;
	
	memcpy(int_out, &double_in, sizeof(float64_t));

	temp1 = int_out[2];
	int_out[0] = int_out[3];	
	int_out[1] = temp1;		
	
	temp2 = int_out[0];
	int_out[2] = int_out[1];	
	int_out[3] = temp2;	
}	

float32_t convert_hex_to_float(uint16_t* in, uint8_t index)
{	
	taskENTER_CRITICAL();
	float32_t out = 0.0;
	
	uint32_t tmp = (in[index] << 16) + in[index+1];	
	memcpy(&out, &tmp, sizeof out);	
	taskEXIT_CRITICAL();
	return out;
	
}


void read_init_settings(void)
{
	
	//Чтение настроек
	status_flash_reg = read_registers_from_flash(&settings[0]);
	
//	//Загружаем "по-умолчанию" если не прочитали 
//	if (status_flash_reg != 0)
//	{
//		for(int i=0; i< REG_COUNT; i++)
//			settings[i] = default_settings[i];			
//	}	
	
	baud_rate_uart_1 = convert_hex_to_float(&settings[0], 65);
	if (baud_rate_uart_1 == 0) baud_rate_uart_1 = 115200;
	
	baud_rate_uart_2 = convert_hex_to_float(&settings[0], 101);
	if (baud_rate_uart_2 == 0) baud_rate_uart_2 = 115200;
	
	baud_rate_uart_3 = convert_hex_to_float(&settings[0], 68);
	if (baud_rate_uart_3 == 0) baud_rate_uart_3 = 11520;
	
	
	//Преобразовываем значения из хранилища настроек в уставки/параметры (номер регистра из regmap - 1):			
	lo_warning_icp = convert_hex_to_float(&settings[0], 2); 	
	hi_warning_icp = convert_hex_to_float(&settings[0], 4); 
	lo_emerg_icp = convert_hex_to_float(&settings[0], 6); 
	hi_emerg_icp = convert_hex_to_float(&settings[0], 8); 
	break_level_icp = convert_hex_to_float(&settings[0], 11); 	
	icp_coef_K = convert_hex_to_float(&settings[0], 15); 
	icp_coef_B = convert_hex_to_float(&settings[0], 17); 
	filter_mode_icp = settings[19];
	range_icp = convert_hex_to_float(&settings[0], 20); 	
	icp_menu_points_for_showing = settings[29]; 	

	trigger_485_ZSK = read_flash(0x8034000); //32 бита
	//trigger_485_ZSK |= read_flash(0x8034004);
	
	size_moving_average_ZSK = settings[33];
	if ( size_moving_average_ZSK < 1 || size_moving_average_ZSK > 512 ) size_moving_average_ZSK = 1;
	
	lo_warning_420 = convert_hex_to_float(&settings[0], 38); 	
	hi_warning_420 = convert_hex_to_float(&settings[0], 40); 	
	lo_emerg_420 = convert_hex_to_float(&settings[0], 42); 	
	hi_emerg_420 = convert_hex_to_float(&settings[0], 44); 			
	down_user_range_4_20 = convert_hex_to_float(&settings[0], 47); 		
	up_user_range_4_20 = convert_hex_to_float(&settings[0], 49); 			
	coef_ampl_420 = convert_hex_to_float(&settings[0], 51); 	
	coef_offset_420 = convert_hex_to_float(&settings[0], 53); 	
	
	
	menu_485_points_for_showing = settings[64];	
	mb_master_timeout = settings[67];		
	
	
	mode_relay = settings[84];
	delay_relay = settings[86];	
	delay_relay_exit = settings[88];	
	
	out_4_20_coef_K = convert_hex_to_float(&settings[0], 90); 
	out_4_20_coef_B = convert_hex_to_float(&settings[0], 92); 
	
	slave_adr = settings[100];	
	warming_up = settings[109];
	
	

	power_supply_warning_lo = convert_hex_to_float(&settings[0], 110);
	power_supply_warning_hi = convert_hex_to_float(&settings[0], 112);
		
	channel_ICP_ON = settings[28];	
	channel_4_20_ON = settings[57];
	channel_485_ON = settings[72];
		
	hart_switch_on = settings[114];	
	hart_slave_address = settings[115];	
	hart_func = settings[116];	
	hart_slave_numreg = settings[117];	
	hart_regs_qty = settings[118];		
	hart_timeout_transmit = settings[119];	
		
	impulse_sign = settings[135];
	hysteresis_TOC = settings[138];
	
}

void vApplicationIdleHook( void )
{

	count_idle++;	
	freeHeapSize = xPortGetFreeHeapSize();	
	HAL_IWDG_Refresh(&hiwdg);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	
		adc_bunch = 1;	
	
		//HAL_ADC_Stop_DMA(&hadc1);
		
		if( Semaphore_Acceleration != NULL )
		{
						static signed portBASE_TYPE xHigherPriorityTaskWoken;
						xHigherPriorityTaskWoken = pdFALSE;	
						xSemaphoreGiveFromISR(Semaphore_Acceleration, &xHigherPriorityTaskWoken);
						if( xHigherPriorityTaskWoken == pdTRUE )
						{
								portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
						}			
						
		}
		
		//conv_complete_1++;
};

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
			adc_bunch = 2; 
			
			//HAL_ADC_Stop_DMA(&hadc1);
					
			if( Semaphore_Acceleration != NULL )
			{
							static signed portBASE_TYPE xHigherPriorityTaskWoken;
							xHigherPriorityTaskWoken = pdFALSE;	
							xSemaphoreGiveFromISR(Semaphore_Acceleration, &xHigherPriorityTaskWoken);
							if( xHigherPriorityTaskWoken == pdTRUE )
							{
									portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
							}			
							
			}	
			
		 //conv_complete_2++;
};

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM7) 
	{		
		
		temp2++;		
		
		if (temp2 >= 10) //1 сек.
		{
				temp1 = count_idle; 
				count_idle = 0;
				temp2 = 0;		
				
				//Таймер для режима обновления загрузчика
				if (bootloader_state == 1)
				{
					if (boot_timer_counter > 10) 
					{
						bootloader_state = 0;	
						worker_status = 0x00;
											
						receiveBuffer[1] = 0x00;
					}
					else boot_timer_counter++;
				}
				else
				{
					boot_timer_counter = 0;
				}
				
				//Таймер для Modbus Master
				timer_485_counter ++;
				if (timer_485_counter > TIME_BREAK_SENSOR_485)
				{
					break_sensor_485 = 1;
				}
				
				
				
				//Таймер для задержки на срабатывание реле 1 (канал 4-20)
				if (flag_delay_relay_1_4_20 == 1)
				{						
					if (timer_delay_relay_1_4_20 == delay_relay)
					{
						relay_permission_1_4_20 = 1;
						timer_delay_relay_1_4_20 = 0;						
					}
					else timer_delay_relay_1_4_20++;										
				}			
				
				//Таймер для задержки на срабатывание реле 2 (канал 4-20)
				if (flag_delay_relay_2_4_20 == 1)
				{						
					if (timer_delay_relay_2_4_20 == delay_relay)
					{
						relay_permission_2_4_20 = 1;
						timer_delay_relay_2_4_20 = 0;						
					}
					else timer_delay_relay_2_4_20++;										
				}
				
				//Таймер для задержки на срабатывание реле 1 (канал icp)
				if (flag_delay_relay_1_icp == 1)
				{						
					if (timer_delay_relay_1_icp == delay_relay)
					{
						relay_permission_1_icp = 1;
						timer_delay_relay_1_icp = 0;						
					}
					else timer_delay_relay_1_icp++;										
				}			
				
				//Таймер для задержки на срабатывание реле 2 (канал icp)
				if (flag_delay_relay_2_icp == 1)
				{						
					if (timer_delay_relay_2_icp == delay_relay)
					{
						relay_permission_2_icp = 1;
						timer_delay_relay_2_icp = 0;						
					}
					else timer_delay_relay_2_icp++;										
				}				

				//Таймер для задержки на срабатывание реле (канал 485)
				for (uint8_t i = 0; i < REG_485_QTY; i++)
				{						
						if (master_delay_relay_array[i].flag_delay_relay_1 == 1)
						{						
							if (master_delay_relay_array[i].timer_delay_relay_1 == delay_relay)
							{
								master_delay_relay_array[i].relay_permission_1 = 1;
								master_delay_relay_array[i].timer_delay_relay_1 = 0;						
							}
							else master_delay_relay_array[i].timer_delay_relay_1++;										
						}			
						
						
						if (master_delay_relay_array[i].flag_delay_relay_2 == 1)
						{						
							if (master_delay_relay_array[i].timer_delay_relay_2 == delay_relay)
							{
								master_delay_relay_array[i].relay_permission_2 = 1;
								master_delay_relay_array[i].timer_delay_relay_2 = 0;						
							}
							else master_delay_relay_array[i].timer_delay_relay_2++;										
						}					
				}
				
				if (quit_relay_button == 1)
				{
					if (quit_timer == QUIT_TIMER) 
					{
						quit_relay_button = 0;
						quit_timer = 0;
					}
					else quit_timer++;
				}
				
		}	
	
		//cpu_load = 100 - (100 * temp1 / 1350);
		cpu_load2 = 100 - (100 * temp1 / 1351854);
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
