/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DIN_Pin GPIO_PIN_13
#define DIN_GPIO_Port GPIOC
#define ADC_420mA_Pin GPIO_PIN_5
#define ADC_420mA_GPIO_Port GPIOA
#define ADC_ICP_Pin GPIO_PIN_6
#define ADC_ICP_GPIO_Port GPIOA
#define ADc_24U_Pin GPIO_PIN_7
#define ADc_24U_GPIO_Port GPIOA
#define SPI3_D_C_Pin GPIO_PIN_6
#define SPI3_D_C_GPIO_Port GPIOC
#define Button_Center_Pin GPIO_PIN_7
#define Button_Center_GPIO_Port GPIOC
#define SD1_Pin GPIO_PIN_8
#define SD1_GPIO_Port GPIOC
#define SD2_Pin GPIO_PIN_9
#define SD2_GPIO_Port GPIOC
#define Button_4_Pin GPIO_PIN_11
#define Button_4_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define SPI3_RESET_Pin GPIO_PIN_11
#define SPI3_RESET_GPIO_Port GPIOC
#define Button_3_Pin GPIO_PIN_3
#define Button_3_GPIO_Port GPIOB
#define REL0_Pin GPIO_PIN_6
#define REL0_GPIO_Port GPIOB
#define REL1_Pin GPIO_PIN_7
#define REL1_GPIO_Port GPIOB
#define Button_2_Pin GPIO_PIN_8
#define Button_2_GPIO_Port GPIOB
#define Button_1_Pin GPIO_PIN_9
#define Button_1_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#define ADC_CHANNEL_NUMBER 2
#define ADC_BUFFER_SIZE 256
#define RAW_ADC_BUFFER_SIZE (ADC_BUFFER_SIZE*4)

#define ADC_BUFFER_SIZE_SMALL 64

//#define QUEUE_LENGHT 32 //2 сек. ( расчет (1/Fcut)*10 ) 
#define QUEUE_LENGHT_4_20 8 //0.5 сек.

#define FILTER_MODE filter_mode_icp

//(Опорное напряжение АЦП / Разрядность АЦП (если oversampling 16 bit)) / Дополнительный коэф.
#define COEF_TRANSFORM_VOLT (3.3 / 4095.0) / (402.0 / 510.0) 

//Диапазон ускорения (м/с2) / диапазон переменного напряжения (Вольты)
//#define COEF_TRANSFORM_icp_acceleration (9.927 / 0.500) 
//#define COEF_TRANSFORM_icp_velocity (20.0 / 0.500)
//#define COEF_TRANSFORM_icp_displacement (40.29 / 0.500)

#define COEF_TRANSFORM_4_20 (20.0 / 4095.0)
#define break_level_4_20 3.7

#define COEF_TRANSFORM_SUPPLY (24.0 / 4095.0) * 1.412

//#define REG_COUNT 744//335 //
#define PAGE 100 //Осн. 0x8032000, резерв 0x8032800 
#define PAGE_ADDR (0x8000000 + (PAGE * 2048))

#define SLAVE_ADR slave_adr
#define SLAVE_BAUDRATE baud_rate_uart_2

#define TIME_BREAK_SENSOR_485 15 //сек.

#define VERSION 5.231

#define REG_485_QTY 40
#define REG_485_START_ADDR 144
#define STRUCTURE_SIZE 20 //Размер структуры для канала 485

#define ZSK_REG_485_QTY 32		 
#define MOVING_AVERAGE 1 //Вкл. усреднение ЗСК

#define TOC_QUEUE_LENGHT (25600 / ADC_BUFFER_SIZE)

#define REG_COUNT (REG_485_START_ADDR + (REG_485_QTY * STRUCTURE_SIZE))

void convert_float_and_swap(float32_t float_in, uint16_t* int_out);
void convert_float_and_swap2(float32_t* float_in, uint16_t* int_out);
float32_t convert_hex_to_float(uint16_t* in, uint8_t index);
void read_init_settings(void);

struct mb_master_delay_relay
{
	uint8_t flag_delay_relay_1;
	uint8_t relay_permission_1;
	uint8_t timer_delay_relay_1;
	uint8_t flag_delay_relay_2;
	uint8_t relay_permission_2;
	uint8_t timer_delay_relay_2;	
};

extern struct mb_master_delay_relay master_delay_relay_array[REG_485_QTY];

#define BUTTON_SENSE 5 //Чувствительность нажатия на кнопку
#define QUIT_TIMER 1 //Таймер нечувствительности (квитирование реле) 

#define BOOT_START_ADDRESS 0x8004000
#define BOOT_CRC_ADR 0x8003000
#define BOOT_SIZE 0x8003020

#define APP_START_ADDRESS 0x8010000
#define APP_CRC_ADR 0x8003800
#define APP_SIZE 0x8003820

#define ZSK_STATE_REG 0x8034000 //(104*2048)

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
