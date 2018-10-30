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

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "arm_math.h"
#include "math.h"
#include <stdint.h>
//#include "Task_manager.h"
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "dac.h"
#include "fonts.h"
#include "ssd1306.h"

//#include "stm32l4xx_it.h"
//#include "modbus_reg_map.h"
//#include "Flash_manager.h"
//#include <string.h>
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

xSemaphoreHandle 	Semaphore1, Semaphore2,
									Semaphore_Acceleration, Semaphore_Velocity, Semaphore_Displacement,
									Q_Semaphore_Acceleration, Q_Semaphore_Velocity, Q_Semaphore_Displacement,
									Semaphore_Modbus_Rx, Semaphore_Modbus_Tx, 
									Semaphore_Master_Modbus_Rx, Semaphore_Master_Modbus_Tx,
									Semaphore_Relay_1, Semaphore_Relay_2,
									Semaphore_HART_Receive, Semaphore_HART_Transmit,
									Mutex_Setting;
									
uint16_t raw_adc_value[RAW_ADC_BUFFER_SIZE];
float32_t float_adc_value_ICP[ADC_BUFFER_SIZE];
float32_t float_adc_value_4_20[ADC_BUFFER_SIZE];
float32_t float_adc_value_ICP_64_1[ADC_BUFFER_SIZE_SMALL];

float32_t dac_voltage = 0.0;
float32_t mean_4_20 = 0.0;

uint64_t xTimeBefore, xTotalTimeSuspended;			

float32_t* Q_A_rms_array_icp;
float32_t* Q_V_rms_array_icp;
float32_t* Q_D_rms_array_icp;
float32_t* Q_A_mean_array_4_20;
float32_t* Q_V_rms_array_4_20;
float32_t* Q_D_rms_array_4_20;

float32_t* Q_A_peak_array_icp;
float32_t* Q_V_peak_array_icp;
float32_t* Q_D_peak_array_icp;
float32_t* Q_A_2peak_array_icp;
float32_t* Q_V_2peak_array_icp;
float32_t* Q_D_2peak_array_icp;

float32_t* Q_peak_array_4_20;
float32_t* Q_2peak_array_4_20;

xQueueHandle acceleration_queue_icp;
xQueueHandle velocity_queue_icp;
xQueueHandle displacement_queue_icp;
xQueueHandle queue_4_20;
xQueueHandle velocity_queue_4_20;
xQueueHandle displacement_queue_4_20;

xQueueHandle acceleration_peak_queue_icp;
xQueueHandle velocity_peak_queue_icp;
xQueueHandle displacement_peak_queue_icp;
xQueueHandle acceleration_2peak_queue_icp;
xQueueHandle velocity_2peak_queue_icp;
xQueueHandle displacement_2peak_queue_icp;

xQueueHandle queue_peak_4_20;
xQueueHandle queue_2peak_4_20;
	
uint8_t queue_count_A_icp;
uint8_t queue_count_A_4_20;
uint8_t queue_count_V_icp;
uint8_t queue_count_V_4_20;
uint8_t queue_count_D_icp;
uint8_t queue_count_D_4_20;

float32_t min_4_20 = 0.0;
float32_t max_4_20 = 0.0;

arm_biquad_casd_df1_inst_f32 filter_main_high_icp;
float32_t pStates_main_high_icp[8];

arm_biquad_casd_df1_inst_f32 filter_main_low_icp;
float32_t pStates_main_low_icp[8];

arm_biquad_casd_df1_inst_f32 filter_main_low_icp_2;
float32_t pStates_main_low_icp_2[8];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_1_icp;
float32_t pStates_highpass_1_icp[8];

arm_biquad_casd_df1_inst_f32 filter_instance_highpass_2_icp;
float32_t pStates_highpass_2_icp[8];


arm_biquad_casd_df1_inst_f32 filter_main_low_4_20;
float32_t pStates_main_low_4_20[8];

arm_biquad_casd_df1_inst_f32 filter_main_high_4_20;
float32_t pStates_main_high_4_20[8];

int16_t settings[REG_COUNT]; //массив настроек 

uint8_t button_state = 0;

uint8_t transmitBuffer[REG_COUNT*2+5];
uint8_t receiveBuffer[16];
uint8_t boot_receiveBuffer[128];
uint8_t master_transmitBuffer[8];
uint8_t master_receiveBuffer[255];
uint8_t HART_receiveBuffer[16];
uint8_t HART_transmitBuffer[8];

uint8_t hart_switch_on = 0;
uint16_t hart_slave_address = 0;
uint16_t hart_slave_numreg = 0;
uint8_t hart_func = 0;
uint16_t hart_regs_qty = 0;
uint16_t hart_timeout_transmit = 100;
uint16_t hart_time_poll = 0;
uint16_t hart_value = 0.0;

//ICP
float32_t icp_voltage = 0.0;
float32_t lo_warning_icp = 0.0;
float32_t hi_warning_icp = 0.0;
float32_t lo_emerg_icp = 0.0;
float32_t hi_emerg_icp = 0.0;
uint8_t break_sensor_icp = 0;
float32_t break_level_icp = 0.0;
float32_t range_icp = 0.0;
uint8_t filter_mode_icp = 0;
float32_t rms_acceleration_icp = 0.0;
float32_t rms_velocity_icp = 0.0;
float32_t rms_displacement_icp = 0.0;

float32_t icp_coef_K = 0.0;
float32_t icp_coef_B = 0.0;

//Амплитуда и размах (ПИК, ПИК-ПИК)
float32_t max_acceleration_icp = 0.0;
float32_t min_acceleration_icp = 0.0;
float32_t max_velocity_icp = 0.0;
float32_t min_velocity_icp = 0.0;
float32_t max_displacement_icp = 0.0;
float32_t min_displacement_icp = 0.0;

//4-20
float32_t current_4_20 = 0.0; //ток входного канала 4-20
float32_t out_required_current = 0.0; //ток для выдачи в выходной канал 4-20
float32_t lo_warning_420 = 0.0;
float32_t hi_warning_420 = 0.0;
float32_t lo_emerg_420 = 0.0;
float32_t hi_emerg_420 = 0.0;
uint8_t break_sensor_420 = 0;
float32_t coef_ampl_420 = 0.0;
float32_t coef_offset_420 = 0.0;

float32_t up_user_range_4_20 = 0.0;
float32_t down_user_range_4_20 = 0.0;
float32_t calculated_value_4_20 = 0.0;

//485
uint16_t slave_adr_mb_master = 0;
uint16_t mb_master_timeout = 0;
uint16_t slave_reg_mb_master = 0;
uint16_t slave_func_mb_master = 0;
float32_t mb_master_recieve_data = 0.0;
uint16_t quantity_reg_mb_master = 0;
uint8_t break_sensor_485 = 0;


struct mb_master
{	
	uint8_t master_on;										//0 
	uint8_t master_addr;									//1 
	uint8_t master_func;									//2 
	uint16_t master_numreg;								//3 
	uint8_t master_type;									//4 
	uint16_t request_timeout;							//5 
	float32_t master_coef_A;							//6,7 
	float32_t master_coef_B;							//8,9 
	float32_t master_value;								//9,10 
	float32_t master_warning_set;					//11,12 
	float32_t master_emergency_set;				//13,14
	float32_t low_master_warning_set;			//15,16 
	float32_t low_master_emergency_set;		//17,18 
};

struct mb_master master_array[REG_485_QTY];
uint8_t master_transmit_buffer[8];
uint8_t master_receive_buffer[9];
uint8_t master_response_received_id = 0;

static TaskHandle_t xTask18 = NULL;

volatile uint64_t mb_master_timeout_error = 0;
volatile float32_t mb_master_timeout_error_percent = 0;
volatile uint64_t mb_master_crc_error = 0;
volatile float32_t mb_master_crc_error_percent = 0;
volatile uint64_t mb_master_request = 0;
volatile uint64_t mb_master_response = 0;

volatile TickType_t xTimeOutBefore, xTotalTimeOutSuspended;

uint16_t trigger_485_event_attribute_warning = 0;
uint16_t trigger_485_event_attribute_emerg = 0;

//Реле
uint8_t state_emerg_relay = 0;
uint8_t state_warning_relay = 0;
uint16_t mode_relay = 0;
uint8_t source_signal_relay = 0;
uint16_t delay_relay = 0;
uint16_t delay_relay_exit = 0;
uint8_t flag_for_delay_relay_exit = 0;
uint16_t warning_relay_counter = 0;
uint16_t emerg_relay_counter = 0;
uint16_t test_relay = 0;

//Реле таймер на срабатывание
uint8_t flag_delay_relay_1_4_20 = 0;
uint8_t relay_permission_1_4_20 = 0;
uint16_t timer_delay_relay_1_4_20 = 0;
uint8_t flag_delay_relay_2_4_20 = 0;
uint8_t relay_permission_2_4_20 = 0;
uint16_t timer_delay_relay_2_4_20 = 0;

uint8_t flag_delay_relay_1_icp = 0;
uint8_t relay_permission_1_icp = 0;
uint16_t timer_delay_relay_1_icp = 0;
uint8_t flag_delay_relay_2_icp = 0;
uint8_t relay_permission_2_icp = 0;
uint16_t timer_delay_relay_2_icp = 0;

//Выход 4-20
uint8_t source_signal_out420 = 0;
float32_t variable_for_out_4_20 = 0.0;	
float32_t out_4_20_coef_K = 0.0;	
float32_t out_4_20_coef_B = 0.0;	

//Дискретный вход
uint8_t bin_input_state = 0;

//Общие
extern float32_t cpu_float;
float32_t power_supply_voltage = 0.0;
uint16_t slave_adr = 0;	
uint16_t warming_up = 0;
uint8_t warming_flag = 1;
float32_t power_supply_warning_lo = 0.0;
float32_t power_supply_warning_hi = 0.0;

//Кнопки
uint8_t button_left = 0;
uint8_t button_right = 0;
uint8_t button_up = 0;
uint8_t button_down = 0;
uint16_t button_center = 0;

uint8_t button_left_pressed_in = 0;
uint8_t button_right_pressed_in = 0;
uint8_t button_up_pressed_in = 0;
uint8_t button_down_pressed_in = 0;
uint8_t button_center_pressed_in_short = 0;
uint8_t button_center_pressed_in_long = 0;

extern FontDef font_7x12_RU;
extern FontDef font_7x12;
extern FontDef font_8x15_RU;
extern FontDef font_8x14;
extern FontDef font_5x10_RU;
extern FontDef font_5x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

uint16_t menu_index = 0;
uint16_t menu_index_array[7];
uint16_t menu_index_pointer = 0;
uint16_t menu_vertical = 0;
uint16_t menu_horizontal = 0;
float32_t baud_rate_uart_2 = 0; //slave
float32_t baud_rate_uart_3 = 0; //master
uint8_t bootloader_state = 0;
extern uint32_t boot_timer_counter;	
uint16_t trigger_event_attribute = 0;


uint16_t channel_ICP_ON = 0;
uint16_t channel_4_20_ON = 0;
uint16_t channel_485_ON = 0;

volatile int temp_var_1 = 0;
volatile int temp_var_2 = 0;

extern uint16_t timer_485_counter;

uint8_t temp_str = 0; //Скроллинг (промотка) строки в меню

uint8_t menu_edit_mode = 0; //Режим редактирования
uint8_t digit_rank = 0; //Разряд числа (для редактирования)
float32_t fractpart = 0.0;

uint8_t temp_stat_1 = 0;
float32_t temp_stat_2 = 0;
uint8_t horizont_menu_lenght = 0;
uint8_t vertical_menu_lenght = 0;
double intpart;	
char buffer[64];
uint8_t config_mode = 0; //Режим конфигурации контроллера

volatile uint16_t number_of_items_in_the_menu = 0;
const uint8_t items_menu_icp = 1;
const uint8_t items_menu_4_20 = 2; 
const uint8_t items_menu_485 = 3;
const uint8_t items_menu_relay = 4;
const uint8_t items_menu_common = 5;
const uint8_t items_menu_info = 6;
const uint8_t items_menu_config = 7;

const uint32_t baudrate_array[] = {1200, 2400, 4800, 9600, 14900, 19200, 38400, 56000, 57600, 115200, 128000, 230400, 256000, 460800, 921600};
uint8_t iter = 0;

uint8_t icp_home_screen_option = 0;

uint16_t reset_to_default = 0;

int16_t icp_menu_points_for_showing = 0;
int16_t menu_485_points_for_showing = 0;
uint8_t menu_edit_settings_mode = 0;

float64_t integrator_summa_V = 0.0;
float64_t integrator_summa_D = 0.0;

//Счетчик оборотов (Turn Over Counter)
uint8_t old_turnover_front = 0;
xQueueHandle queue_TOC;
uint16_t queue_count_TOC;
volatile float32_t turnover_count_1s = 0.0;		
volatile float32_t turnover_count_60s = 0.0;
volatile uint16_t pass_count = 0;
volatile uint8_t turnover_front = 0;	
volatile uint64_t big_points_counter = 0;
volatile uint64_t small_points_counter = 0;
volatile uint64_t difference_points_counter = 0;
volatile float32_t common_level_TOC = 0.0;
volatile float32_t mean_level_TOC = 0.0;
volatile float32_t level_summa_TOC = 0.0;
volatile float32_t turnover_summa[TOC_QUEUE_LENGHT];
uint16_t summa_iter = 0;
uint16_t impulse_sign = 0;
uint16_t hysteresis_TOC = 0;

struct mb_master_delay_relay master_delay_relay_array[REG_485_QTY];

uint8_t QUEUE_LENGHT = 32;

uint8_t quit_relay_button = 0;

volatile uint8_t disable_up_down_button = 0; //Флаг для запрета кнопок ввех и вниз

volatile uint8_t adc_bunch = 0; //Индекс половинки буфера АЦП

volatile uint16_t bunch_count_1 = 0;
volatile uint16_t bunch_count_2 = 0;


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
void FilterInit(void);
void Integrate_V(float32_t* input, float32_t* output, uint32_t size);
void Integrate_D(float32_t* input, float32_t* output, uint32_t size);
extern void write_flash(uint32_t page, uint32_t* data, uint32_t size);
extern uint32_t read_flash(uint32_t addr);
extern uint16_t crc16(uint8_t *adr_buffer, uint32_t byte_cnt);
uint16_t crc_calculating(unsigned char* puchMsg, unsigned short usDataLen);
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
uint32_t rtc_read_backup_reg(uint32_t BackupRegister);
void rtc_write_backup_reg(uint32_t BackupRegister, uint32_t data);
void string_scroll(char* msg, uint8_t len);
void string_scroll_with_number(char* msg, uint8_t len, uint8_t number);
void edit_mode(float32_t *var);
void edit_mode_int(int16_t *var); 
void edit_mode_int8(uint8_t *var); 
void init_menu(uint8_t where_from);
void save_settings(void);
void edit_mode_from_list(float32_t *var, uint32_t* list);
void turnover_counter(float32_t* input_array);
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
	char msg[30];
	uint16_t temp_buf[2];
	
	// CS# (This pin is the chip select input. (active LOW))
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	
	ssd1306_Init();
	ssd1306_Fill(1);
	check_logo();
	ssd1306_UpdateScreen();
	//osDelay(settings[109]/2); //Заливка половина времени прогрева
	osDelay(1000);

	init_menu(1);
	
  /* Infinite loop */
  for(;;)
  {		
		
//			if (warming_flag == 1) 
//			{				
				ssd1306_Fill(0);				
				
				logo();
				
				ssd1306_UpdateScreen();
//			}
//			else 
//			{							
//					//Навигация по горизонтальному меню							
//					if (menu_index_pointer == 1) //ICP
//					{
//						if (menu_edit_settings_mode == 0) horizont_menu_lenght = 9; 
//						else horizont_menu_lenght = 6;
//					}
//					
//					if (menu_index_pointer == 2) //4-20
//					{						
//						if (menu_edit_settings_mode == 0) horizont_menu_lenght = 1; 
//						else horizont_menu_lenght = 8;
//					}
//					
//					if (menu_index_pointer == 3) //485
//					{
//						if (menu_edit_settings_mode == 0) horizont_menu_lenght = REG_485_QTY;
//						else horizont_menu_lenght = 2;	
//					}
//					
//					if (menu_index_pointer == 4) //Реле
//					{						
//						if (menu_edit_settings_mode == 0) horizont_menu_lenght = 3;
//						else horizont_menu_lenght = 4;	
//					}
//					
//					if (menu_index_pointer == 5) horizont_menu_lenght = 4; //Настройки
//					if (menu_index_pointer == 6) horizont_menu_lenght = 3; //Информация
//					if (menu_index_pointer == 7) horizont_menu_lenght = 3; //Конфигурация
//					
//				
//					if (button_left_pressed_in == 1 && menu_horizontal > 0 && menu_edit_mode == 0) 
//					{				
//						menu_horizontal--;
//						button_left_pressed_in = 0;
//						button_center_pressed_in_short = 0;				
//						digit_rank = 0;						
//					}						
//					
//					if (button_right_pressed_in == 1 && menu_horizontal < horizont_menu_lenght && menu_edit_mode == 0) 					
//					{				
//						menu_horizontal++;
//						button_right_pressed_in = 0;
//						button_center_pressed_in_short = 0;
//						digit_rank = 0;						
//					}	
//					
//					//Навигация по вертикальному меню
//					if (button_up_pressed_in == 1 && menu_index_pointer > 0 && button_center_pressed_in_short == 0 && menu_edit_mode == 0) 										
//					{				
//						
//						if (menu_index_pointer == 3 && menu_horizontal != 0) //меню 485
//						{
//								if (menu_vertical > 0) menu_vertical--;
//								button_up_pressed_in = 0;
//								digit_rank = 0;
//						}
//						else
//						{						
//								if (menu_index > 0) menu_index--;					
//								menu_index_pointer = menu_index_array[menu_index];
//								
//								button_up_pressed_in = 0;						
//								menu_horizontal = 0;					
//								digit_rank = 0;						
//						}
//					}						
//						
//					if (button_down_pressed_in == 1 && button_center_pressed_in_short == 0 && menu_edit_mode == 0) 					
//					{						

//						if (menu_index_pointer == 3 && menu_horizontal != 0) //меню 485
//						{
//									if (menu_vertical < REG_485_QTY && menu_vertical < 12) menu_vertical++;
//									button_down_pressed_in = 0;
//									digit_rank = 0;
//						}						
//						else
//						{
//								if (menu_index < number_of_items_in_the_menu-1) menu_index++;
//								menu_index_pointer = menu_index_array[menu_index];		
//								
//								button_down_pressed_in = 0;																		
//								menu_horizontal = 0;						
//								digit_rank = 0;						
//						}
//					}	
//					
//					//При коротком нажатии включаем/выключаем режим редактирования, но не в гл.меню
//					if (button_center_pressed_in_short == 1 && menu_horizontal != 0) 
//					{
//						menu_edit_mode = !menu_edit_mode;	
//						button_center_pressed_in_short = 0;						
//					}					
//					//При длинном нажатии в гл.меню включаем/выключаем настроечный режим  
//					else if (button_center_pressed_in_long == 1 && menu_horizontal == 0) 
//					{
//						menu_edit_settings_mode = !menu_edit_settings_mode;	
//						button_center_pressed_in_long = 0;				
//						quit_relay_button = 1; //Включаем таймер чтоб не срабатывало квитирование 						
//					}
//					
//					//Переход между разрядами числа в режиме редактирования
//					if (button_left_pressed_in == 1 && menu_edit_mode == 1) 
//					{				
//						if (digit_rank > 0) digit_rank--;
//						else digit_rank = 0;
//						
//						button_left_pressed_in = 0;	
//					}						
//					
//					if (button_right_pressed_in == 1 && menu_edit_mode == 1) 					
//					{				
//						if (digit_rank < 1) digit_rank++;
//						else digit_rank = 1;
//						
//						button_right_pressed_in = 0;						
//					}	
//					
//					//Сохранение настроек на флеш
//					if (button_center_pressed_in_long == 1 && menu_horizontal != 0)
//					{
//						save_settings();
//						button_center_pressed_in_long = 0;
//						button_center_pressed_in_short = 0;
//						menu_edit_settings_mode = 0;
//						quit_relay_button = 1; //Включаем таймер чтоб не срабатывало квитирование						
//					}
//					
//					
//					
////////////ICP menu					
//					if (channel_ICP_ON == 1)
//					{	
//							if (menu_index_pointer == 1 && menu_horizontal == 0)
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);		
//								
//								ssd1306_WriteString("ICP",font_8x14,1);										
//														
//								if (break_sensor_icp == 1) //Если обрыв
//								{									
//										if (temp_stat_1 == 0) 
//										{
//											ssd1306_SetCursor(0,15);											
//											ssd1306_WriteString("ОБРЫВ",font_8x15_RU,1);
//											ssd1306_SetCursor(0,30);	
//											ssd1306_WriteString("ДАТЧИКА",font_8x15_RU,1);
//										}
//										else ssd1306_WriteString(" ",font_8x14,1);
//										
//										if (menu_edit_settings_mode == 1) triangle_right(55,0);
//										triangle_right(60,0);																					
//										triangle_down(58,43);
//								}
//								else
//								{

//									if (menu_edit_settings_mode == 0) //Режим просмотра вибропараметров ">"
//									{
//										ssd1306_Fill(0);
//										ssd1306_SetCursor(0,0);												
//										ssd1306_WriteString("ICP",font_8x14,1);										
//										ssd1306_SetCursor(28,0);																														

//										triangle_right(60,0);																					
//										triangle_down(58,43);
//										
//										ssd1306_SetCursor(0,15);																									
//									}
//									else if (menu_edit_settings_mode == 1) //Режим настройки канала ">>"
//									{
//										ssd1306_Fill(0);
//										ssd1306_SetCursor(0,0);												
//										ssd1306_WriteString("ICP",font_8x14,1);										
//										ssd1306_SetCursor(28,0);																														

//										triangle_right(55,0);
//										triangle_right(60,0);																					
//										triangle_down(58,43);
//										
//										ssd1306_SetCursor(0,15);
//									}										
//									

//									if (icp_menu_points_for_showing == 1)	
//									{
//										strncpy(msg,"СКЗ виброускорения", 18);
//										string_scroll(msg, 18);								
//										ssd1306_SetCursor(0,32);				
//										snprintf(buffer, sizeof buffer, "%.01f", rms_acceleration_icp);
//										ssd1306_WriteString(buffer,font_8x14,1);											
//									}
//									
//									if (icp_menu_points_for_showing == 2)	
//									{
//										strncpy(msg,"СКЗ виброскорости", 17);
//										string_scroll(msg, 17);									
//										ssd1306_SetCursor(0,32);				
//										snprintf(buffer, sizeof buffer, "%.01f", rms_velocity_icp);
//										ssd1306_WriteString(buffer,font_8x14,1);		
//									}										

//									if (icp_menu_points_for_showing == 3)	
//									{
//										strncpy(msg,"СКЗ виброперемещения", 20);
//										string_scroll(msg, 20);								
//										ssd1306_SetCursor(0,32);				
//										snprintf(buffer, sizeof buffer, "%.01f", rms_displacement_icp);
//										ssd1306_WriteString(buffer,font_8x14,1);											
//									}			

//									if (icp_menu_points_for_showing == 4)	
//									{
//										strncpy(msg,"Амплитуда виброускорения", 20);
//										string_scroll(msg, 20);								
//										ssd1306_SetCursor(0,32);				
//										snprintf(buffer, sizeof buffer, "%.01f", max_acceleration_icp);
//										ssd1306_WriteString(buffer,font_8x14,1);											
//									}	
//									
//									if (icp_menu_points_for_showing == 5)	
//									{
//										strncpy(msg,"Амплитуда виброскорости", 23);
//										string_scroll(msg, 23);								
//										ssd1306_SetCursor(0,32);				
//										snprintf(buffer, sizeof buffer, "%.01f", max_velocity_icp);
//										ssd1306_WriteString(buffer,font_8x14,1);											
//									}										

//									if (icp_menu_points_for_showing == 6)	
//									{
//										strncpy(msg,"Амплитуда виброперемещения", 20);
//										string_scroll(msg, 20);								
//										ssd1306_SetCursor(0,32);				
//										snprintf(buffer, sizeof buffer, "%.01f", max_displacement_icp);
//										ssd1306_WriteString(buffer,font_8x14,1);											
//									}		
//									
//									
//									if (icp_menu_points_for_showing == 7)	
//									{
//										strncpy(msg,"Размах виброускорения", 20);
//										string_scroll(msg, 20);								
//										ssd1306_SetCursor(0,32);				
//										snprintf(buffer, sizeof buffer, "%.01f", max_acceleration_icp - min_acceleration_icp);
//										ssd1306_WriteString(buffer,font_8x14,1);											
//									}	
//									
//									if (icp_menu_points_for_showing == 8)	
//									{
//										strncpy(msg,"Размах виброскорости", 20);
//										string_scroll(msg, 20);								
//										ssd1306_SetCursor(0,30);				
//										snprintf(buffer, sizeof buffer, "%.01f", max_velocity_icp - min_velocity_icp);
//										ssd1306_WriteString(buffer,font_8x14,1);											
//									}										

//									if (icp_menu_points_for_showing == 9)	
//									{
//										strncpy(msg,"Размах виброперемещения", 20);
//										string_scroll(msg, 20);								
//										ssd1306_SetCursor(0,32);				
//										snprintf(buffer, sizeof buffer, "%.01f", max_displacement_icp - min_displacement_icp);
//										ssd1306_WriteString(buffer,font_8x14,1);											
//									}
//								}								
//																
//								//ssd1306_UpdateScreen();				
//								
//								menu_edit_mode = 0 ; //Запрещаем редактирование
//								
//								disable_up_down_button = 0;
//							}

//							
//							if (menu_index_pointer == 1 && menu_horizontal == 1 && menu_edit_settings_mode == 0)
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);										
//								ssd1306_SetCursor(28,0);						
//								triangle_left(55,0);						
//								triangle_right(60,0);						
//								
//								ssd1306_SetCursor(0,15);											
//								strncpy(msg,"СКЗ виброускорения", 18);
//								string_scroll(msg, 18);
//								
//								ssd1306_SetCursor(0,32);				
//								snprintf(buffer, sizeof buffer, "%.03f", rms_acceleration_icp);
//								ssd1306_WriteString(buffer,font_8x14,1);							
//								
//								//ssd1306_UpdateScreen();			

//								menu_edit_mode = 0 ; //Запрещаем редактирование				
//								
//								disable_up_down_button = 1; //Выключаем кнопки вверх вниз

//							}						

//							

//							if (menu_index_pointer == 1 && menu_horizontal == 2 && menu_edit_settings_mode == 0)
//							{								
//									
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);										
//								ssd1306_SetCursor(28,0);																		
//								triangle_left(55,0);						
//								triangle_right(60,0);						
//								
//								ssd1306_SetCursor(0,15);																																		
//								strncpy(msg,"СКЗ виброскорости", 17);
//								string_scroll(msg, 17);
//									
//								ssd1306_SetCursor(0,32);				
//								snprintf(buffer, sizeof buffer, "%.03f", rms_velocity_icp);
//								ssd1306_WriteString(buffer,font_8x14,1);		
//								
//								//ssd1306_UpdateScreen();			

//								menu_edit_mode = 0 ; //Запрещаем редактирование		
//								
//								disable_up_down_button = 1; //Выключаем кнопки вверх вниз

//							}			

//					
//							if (menu_index_pointer == 1 && menu_horizontal == 3 && menu_edit_settings_mode == 0)
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);										
//								ssd1306_SetCursor(28,0);						
//								triangle_left(55,0);						
//								triangle_right(60,0);					
//								
//								ssd1306_SetCursor(0,15);											
//								strncpy(msg,"СКЗ виброперемещения", 20);
//								string_scroll(msg, 20);
//								
//								ssd1306_SetCursor(0,32);				
//								snprintf(buffer, sizeof buffer, "%.03f", rms_displacement_icp);
//								ssd1306_WriteString(buffer,font_8x14,1);							
//								
//								//ssd1306_UpdateScreen();			

//								menu_edit_mode = 0 ; //Запрещаем редактирование

//								disable_up_down_button = 1; //Выключаем кнопки вверх вниз								

//							}
//							
//							if (menu_index_pointer == 1 && menu_horizontal == 4 && menu_edit_settings_mode == 0) //Амплитуда ускорения
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);	
//								triangle_left(55,0);						
//								triangle_right(60,0);				
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Амплитуда виброускорения", 24);						
//								string_scroll(msg, 24);
//								
//								ssd1306_SetCursor(0,32);				
//								snprintf(buffer, sizeof buffer, "%.03f", max_acceleration_icp);
//								ssd1306_WriteString(buffer,font_8x14,1); 		
//														
//								//ssd1306_UpdateScreen();

//								menu_edit_mode = 0 ; //Запрещаем редактирование		

//								disable_up_down_button = 1; //Выключаем кнопки вверх вниз
//							}							
//							
//							if (menu_index_pointer == 1 && menu_horizontal == 5 && menu_edit_settings_mode == 0) //Амплитуда скорости
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);	
//								triangle_left(55,0);						
//								triangle_right(60,0);					
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Амплитуда виброскорости", 23);						
//								string_scroll(msg, 23);
//								
//								ssd1306_SetCursor(0,32);				
//								snprintf(buffer, sizeof buffer, "%.03f", max_velocity_icp);
//								ssd1306_WriteString(buffer,font_8x14,1); 		
//														
//								//ssd1306_UpdateScreen();

//								menu_edit_mode = 0 ; //Запрещаем редактирование		

//								disable_up_down_button = 1; //Выключаем кнопки вверх вниз
//							}

//							if (menu_index_pointer == 1 && menu_horizontal == 6 && menu_edit_settings_mode == 0) //Амплитуда перемещения
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);	
//								triangle_left(55,0);						
//								triangle_right(60,0);			
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Амплитуда виброперемещения", 26);						
//								string_scroll(msg, 26);
//								
//								ssd1306_SetCursor(0,32);				
//								snprintf(buffer, sizeof buffer, "%.03f", max_displacement_icp);
//								ssd1306_WriteString(buffer,font_8x14,1); 		
//														
//								//ssd1306_UpdateScreen();

//								menu_edit_mode = 0 ; //Запрещаем редактирование		

//								disable_up_down_button = 1; //Выключаем кнопки вверх вниз
//							}

//							if (menu_index_pointer == 1 && menu_horizontal == 7 && menu_edit_settings_mode == 0) //Размах ускорения
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);	
//								triangle_left(55,0);						
//								triangle_right(60,0);				
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Размах виброускорения", 21);						
//								string_scroll(msg, 21);
//								
//								ssd1306_SetCursor(0,32);				
//								snprintf(buffer, sizeof buffer, "%.03f", max_acceleration_icp - min_acceleration_icp);
//								ssd1306_WriteString(buffer,font_8x14,1); 		
//														
//								//ssd1306_UpdateScreen();

//								menu_edit_mode = 0 ; //Запрещаем редактирование	

//								disable_up_down_button = 1; //Выключаем кнопки вверх вниз
//							}						

//							
//							if (menu_index_pointer == 1 && menu_horizontal == 8 && menu_edit_settings_mode == 0) //Размах скорости
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);	
//								triangle_left(55,0);						
//								triangle_right(60,0);				
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Размах виброскорости", 20);						
//								string_scroll(msg, 20);
//								
//								ssd1306_SetCursor(0,32);				
//								snprintf(buffer, sizeof buffer, "%.03f", max_velocity_icp - min_velocity_icp);
//								ssd1306_WriteString(buffer,font_8x14,1); 		
//														
//								//ssd1306_UpdateScreen();

//								menu_edit_mode = 0 ; //Запрещаем редактирование

//								disable_up_down_button = 1; //Выключаем кнопки вверх вниз								
//							}				

//							
//							if (menu_index_pointer == 1 && menu_horizontal == 9 && menu_edit_settings_mode == 0) //Размах перемещения
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);	
//								triangle_left(55,0);						
//																							
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Размах виброперемещения", 23);						
//								string_scroll(msg, 23);
//								
//								ssd1306_SetCursor(0,32);				
//								snprintf(buffer, sizeof buffer, "%.03f", max_displacement_icp - min_displacement_icp);
//								ssd1306_WriteString(buffer,font_8x14,1); 		
//														
//								//ssd1306_UpdateScreen();	

//								menu_edit_mode = 0 ; //Запрещаем редактирование

//								disable_up_down_button = 1; //Выключаем кнопки вверх вниз								
//							}							




//							//Режим настройки ICP
//							
//							if (menu_index_pointer == 1 && menu_horizontal == 1 && menu_edit_settings_mode == 1) //Номер параметра для показа на гл. экране
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);												
//								triangle_left(50,0);
//								triangle_right(55,0);							
//								triangle_right(60,0);		
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Параметр на главном меню", 24);						
//								string_scroll(msg, 24);
//								
//								ssd1306_SetCursor(0,32);			
//								
//								if (menu_edit_mode == 1) //Режим редактирования
//								{											
//											edit_mode_int(&icp_menu_points_for_showing);			
//											disable_up_down_button = 0;
//								}
//								else //Нормальный режим
//								{
//									snprintf(buffer, sizeof buffer, "%d", icp_menu_points_for_showing);
//									ssd1306_WriteString(buffer,font_8x14,1); 
//									
//									disable_up_down_button = 1; //Выключаем кнопки вверх вниз
//								}												
//								
//								//ssd1306_UpdateScreen();			
//								
//							}
//							
//							
//							if (menu_index_pointer == 1 && menu_horizontal == 2 && menu_edit_settings_mode == 1) //Предупр. уставка
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);												
//								triangle_left(50,0);
//								triangle_right(55,0);							
//								triangle_right(60,0);		
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Предупредительная уставка", 25);						
//								string_scroll(msg, 25);
//								
//								ssd1306_SetCursor(0,32);			
//								
//								if (menu_edit_mode == 1) //Режим редактирования
//								{
//											
//											edit_mode(&hi_warning_icp);
//											disable_up_down_button = 0;
//										
//								}
//								else //Нормальный режим
//								{
//									snprintf(buffer, sizeof buffer, "%.01f", hi_warning_icp);
//									ssd1306_WriteString(buffer,font_8x14,1); 
//									
//									disable_up_down_button = 1; //Выключаем кнопки вверх вниз
//								}												
//								
//								//ssd1306_UpdateScreen();				
//							}					
//							
//							
//							
//							if (menu_index_pointer == 1 && menu_horizontal == 3 && menu_edit_settings_mode == 1) //Авар. уставка
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);	
//								triangle_left(50,0);
//								triangle_right(55,0);							
//								triangle_right(60,0);					
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Аварийная уставка", 17);						
//								string_scroll(msg, 17);						
//								ssd1306_SetCursor(0,32);

//								if (menu_edit_mode == 1) //Режим редактирования
//								{
//									edit_mode(&hi_emerg_icp);
//									disable_up_down_button = 0;
//								}
//								else //Нормальный режим
//								{
//									snprintf(buffer, sizeof buffer, "%.01f", hi_emerg_icp);
//									ssd1306_WriteString(buffer,font_8x14,1); 
//									
//									disable_up_down_button = 1; //Выключаем кнопки вверх вниз
//								}					
//														
//								//ssd1306_UpdateScreen();				
//							}	
//							
//							
//							
//							if (menu_index_pointer == 1 && menu_horizontal == 4 && menu_edit_settings_mode == 1) //Режим цифрового фильтра
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);	
//								triangle_left(50,0);
//								triangle_right(55,0);							
//								triangle_right(60,0);	
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Режим фильтра", 13);						
//								string_scroll(msg, 13);
//								
//								ssd1306_SetCursor(0,32);										
//								
////								if (menu_edit_mode == 1) //Режим редактирования
////								{
////									edit_mode_int8(&filter_mode_icp);
////									disable_up_down_button = 0;
////								}
////								else //Нормальный режим
//								
//								{
//									snprintf(buffer, sizeof buffer, "%d", filter_mode_icp);
//									ssd1306_WriteString(buffer,font_8x14,1); 
//									
//									disable_up_down_button = 1; //Выключаем кнопки вверх вниз
//								}					
//														
//								//ssd1306_UpdateScreen();				
//							}							

//							if (menu_index_pointer == 1 && menu_horizontal == 5 && menu_edit_settings_mode == 1) //Коэф. усиления
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);	
//								triangle_left(50,0);
//								triangle_right(55,0);							
//								triangle_right(60,0);	
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Коэффициент усиления", 20);						
//								string_scroll(msg, 20);
//								
//								ssd1306_SetCursor(0,32);				
//								snprintf(buffer, sizeof buffer, "%.05f", icp_coef_K);										
//								ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//														
//								//ssd1306_UpdateScreen();				
//								
//								menu_edit_mode = 0 ; //Запрещаем редактирование					

//								disable_up_down_button = 1; //Выключаем кнопки вверх вниз								
//							}	

//							
//							if (menu_index_pointer == 1 && menu_horizontal == 6 && menu_edit_settings_mode == 1) //Коэф. смещения
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("ICP",font_8x14,1);	
//								triangle_left(50,0);
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Коэффициент смещения", 20);						
//								string_scroll(msg, 20);
//								
//								ssd1306_SetCursor(0,32);				
//								snprintf(buffer, sizeof buffer, "%.05f", icp_coef_B);										
//								ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//														
//								//ssd1306_UpdateScreen();

//								menu_edit_mode = 0 ; //Запрещаем редактирование			

//								disable_up_down_button = 1; //Выключаем кнопки вверх вниз								
//							}

//					}
//					
////////////4-20 menu		
//					if (channel_4_20_ON == 1)
//					{					
//					
//							if (menu_index_pointer == 2 && menu_horizontal == 0) 
//							{
//								
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("4-20",font_8x14,1);										
//														
//								if (break_sensor_420 == 1 & channel_4_20_ON == 1) //Символ обрыва
//								{							
//										if (temp_stat_1 == 0) 
//										{
//											ssd1306_SetCursor(0,15);											
//											ssd1306_WriteString("ОБРЫВ",font_8x15_RU,1);
//											ssd1306_SetCursor(0,30);	
//											ssd1306_WriteString("ДАТЧИКА",font_8x15_RU,1);
//										}
//										else ssd1306_WriteString(" ",font_8x14,1);
//										
//										if (menu_edit_settings_mode == 1) triangle_right(55,0);
//										triangle_right(60,0);											
//										if (channel_ICP_ON == 1) triangle_up(58,38);											
//										triangle_down(58,43);
//								}
//								else
//								{
//								
//									if (menu_edit_settings_mode == 0)
//									{										
//										triangle_right(60,0);											
//										if (channel_ICP_ON == 1) triangle_up(58,38);											
//										triangle_down(58,43);
//									}
//									else		
//									{
//										triangle_right(55,0);
//										triangle_right(60,0);											
//										if (channel_ICP_ON == 1) triangle_up(58,38);											
//										triangle_down(58,43);
//									}	
//									
//									ssd1306_SetCursor(0,15);																									
//									
//																		
//									strncpy(msg,"Расчетное значение", 18);						
//									string_scroll(msg, 18);
//									
//									ssd1306_SetCursor(0,32);				
//									
//									snprintf(buffer, sizeof buffer, "%.03f", calculated_value_4_20);
//									ssd1306_WriteString(buffer,font_8x14,1);							
//									
//								}
//								//ssd1306_UpdateScreen();			

//								disable_up_down_button = 0;								
//							}			

//							if (menu_index_pointer == 2 && menu_horizontal == 1 && menu_edit_settings_mode == 0)							
//							{
//								ssd1306_Fill(0);

//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("4-20",font_8x14,1);	
//								triangle_left(55,0);				
//								
//								ssd1306_SetCursor(0,15);	
//								ssd1306_WriteString("Ток",font_8x15_RU,1);		

//								ssd1306_SetCursor(0,32);
//								snprintf(buffer, sizeof buffer, "%.02f", mean_4_20);
//								ssd1306_WriteString(buffer,font_8x14,1);
//								//ssd1306_UpdateScreen();				
//								menu_edit_mode = 0 ; //Запрещаем редактирование			

//								disable_up_down_button = 1;								
//							}

//						
//							
//							//Режим настройки канала 4-20
//							if (menu_index_pointer == 2 && menu_horizontal == 1 && menu_edit_settings_mode == 1) //Нижний диапазон
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("4-20",font_8x14,1);	
//								triangle_left(50,0);						
//								triangle_right(55,0);							
//								triangle_right(60,0);					
//								
//								ssd1306_SetCursor(0,15);	
//								strncpy(msg,"Нижний предел диапазона для пересчета", 37);						
//								string_scroll(msg, 37);
//								
//								ssd1306_SetCursor(0,32);				
//								
//								
//								if (menu_edit_mode == 1) //Режим редактирования
//								{								
//									edit_mode(&down_user_range_4_20);
//									disable_up_down_button = 0;
//								}
//								else 
//								{
//									snprintf(buffer, sizeof buffer, "%.01f", down_user_range_4_20);
//									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//									disable_up_down_button = 1;
//								}
//														
//								ssd1306_UpdateScreen();				
//							}	
//							
//							if (menu_index_pointer == 2 && menu_horizontal == 2 && menu_edit_settings_mode == 1) //Верхний диапазон
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("4-20",font_8x14,1);	
//								triangle_left(50,0);						
//								triangle_right(55,0);							
//								triangle_right(60,0);			
//								
//								ssd1306_SetCursor(0,15);	
//								strncpy(msg,"Верхний предел диапазона для пересчета", 38);						
//								string_scroll(msg, 38);
//								
//								ssd1306_SetCursor(0,32);				
//								
//								
//								if (menu_edit_mode == 1) //Режим редактирования
//								{								
//									edit_mode(&up_user_range_4_20);
//									disable_up_down_button = 0;
//								}
//								else 
//								{
//									snprintf(buffer, sizeof buffer, "%.01f", up_user_range_4_20);
//									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//									disable_up_down_button = 1;
//								}
//														
//								ssd1306_UpdateScreen();				
//							}	
//							
//							
//							
//							if (menu_index_pointer == 2 && menu_horizontal == 3 && menu_edit_settings_mode == 1) //Уставка нижняя предупр.
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("4-20",font_8x14,1);	
//								triangle_left(50,0);						
//								triangle_right(55,0);							
//								triangle_right(60,0);		
//								
//								ssd1306_SetCursor(0,15);	
//								strncpy(msg,"Уставка нижняя предупредительная", 32);						
//								string_scroll(msg, 32);
//								
//								ssd1306_SetCursor(0,32);				
//								
//								
//								if (menu_edit_mode == 1) //Режим редактирования
//								{								
//									edit_mode(&lo_warning_420);
//									disable_up_down_button = 0;
//								}
//								else 
//								{
//									snprintf(buffer, sizeof buffer, "%.01f", lo_warning_420);
//									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//									disable_up_down_button = 1;
//								}
//														
//								//ssd1306_UpdateScreen();				
//							}		
//							
//							
//							if (menu_index_pointer == 2 && menu_horizontal == 4 && menu_edit_settings_mode == 1) //Уставка нижняя авар.
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("4-20",font_8x14,1);	
//								triangle_left(50,0);						
//								triangle_right(55,0);							
//								triangle_right(60,0);									
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Уставка нижняя аварийная", 24);						
//								string_scroll(msg, 24);						
//								
//								ssd1306_SetCursor(0,32);				
//												
//								
//								if (menu_edit_mode == 1) //Режим редактирования
//								{
//									edit_mode(&lo_emerg_420);
//									disable_up_down_button = 0;
//								}
//								else 
//								{
//									snprintf(buffer, sizeof buffer, "%.01f", lo_emerg_420);
//									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//									disable_up_down_button = 1;
//								}
//														
//								//ssd1306_UpdateScreen();				
//							}					

//							if (menu_index_pointer == 2 && menu_horizontal == 5 && menu_edit_settings_mode == 1) //Уставка врехняя предупр.
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("4-20",font_8x14,1);	
//								triangle_left(50,0);						
//								triangle_right(55,0);							
//								triangle_right(60,0);		
//								
//								ssd1306_SetCursor(0,15);	
//								strncpy(msg,"Уставка верхняя предупредительная", 33);						
//								string_scroll(msg, 33);
//								
//								ssd1306_SetCursor(0,32);				
//														
//								if (menu_edit_mode == 1) //Режим редактирования
//								{
//									edit_mode(&hi_warning_420);
//									disable_up_down_button = 0;
//								}
//								else 
//								{
//									snprintf(buffer, sizeof buffer, "%.01f", hi_warning_420);			
//									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//									disable_up_down_button = 1;
//								}
//														
//								//ssd1306_UpdateScreen();				
//							}

//							if (menu_index_pointer == 2 && menu_horizontal == 6 && menu_edit_settings_mode == 1) //Уставка врехняя авар.
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("4-20",font_8x14,1);	
//								triangle_left(50,0);						
//								triangle_right(55,0);							
//								triangle_right(60,0);		
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Уставка верхняя аварийная", 25);						
//								string_scroll(msg, 25);
//								
//								ssd1306_SetCursor(0,32);				
//								
//								if (menu_edit_mode == 1) //Режим редактирования
//								{
//									edit_mode(&hi_emerg_420);
//									disable_up_down_button = 0;
//								}
//								else 
//								{
//									snprintf(buffer, sizeof buffer, "%.01f", hi_emerg_420);			
//									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//									disable_up_down_button = 1;
//								}
//														
//								//ssd1306_UpdateScreen();				
//							}					
//							
//							
//							if (menu_index_pointer == 2 && menu_horizontal == 7 && menu_edit_settings_mode == 1)							
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("4-20",font_8x14,1);	
//								triangle_left(50,0);						
//								triangle_right(55,0);							
//								triangle_right(60,0);				
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Коэффициент усиления", 20);						
//								string_scroll(msg, 20);		
//								
//								ssd1306_SetCursor(0,30);
//								snprintf(buffer, sizeof buffer, "%.05f", coef_ampl_420);
//								ssd1306_WriteString(buffer,font_8x14,1);
//								//ssd1306_UpdateScreen();								
//								
//								menu_edit_mode = 0 ; //Запрещаем редактирование
//								
//								disable_up_down_button = 1;
//							}

//							if (menu_index_pointer == 2 && menu_horizontal == 8 && menu_edit_settings_mode == 1)							
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("4-20",font_8x14,1);	
//								triangle_left(50,0);						
//																
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Коэффициент смещения", 20);						
//								string_scroll(msg, 20);		
//								
//								ssd1306_SetCursor(0,30);
//								snprintf(buffer, sizeof buffer, "%.05f", coef_offset_420);
//								ssd1306_WriteString(buffer,font_8x14,1);
//								//ssd1306_UpdateScreen();				

//								menu_edit_mode = 0 ; //Запрещаем редактирование		

//								disable_up_down_button = 1;								
//							}								
//						
//					}
//					
////////////485 menu		
//					if (channel_485_ON == 1)
//					{
//						
//							

//							if (menu_index_pointer == 3 && menu_horizontal == 0) //Значение регистра
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("485",font_8x14,1);		
//								
//								if (menu_edit_settings_mode == 0)
//								{									
//									triangle_right(60,0);											
//									if (channel_ICP_ON == 1 || channel_4_20_ON == 1) triangle_up(58,38);											
//									triangle_down(58,43);
//								}
//								else		
//								{
//									triangle_right(55,0);
//									triangle_right(60,0);											
//									if (channel_ICP_ON == 1 || channel_4_20_ON == 1) triangle_up(58,38);											
//									triangle_down(58,43);
//								}								
//														
//								if (break_sensor_485 == 1) //Символ обрыва
//								{							
//										if (temp_stat_1 == 0) 
//										{
//											ssd1306_SetCursor(0,15);											
//											ssd1306_WriteString("НЕТ",font_8x15_RU,1);
//											ssd1306_SetCursor(0,30);	
//											ssd1306_WriteString("СВЯЗИ",font_8x15_RU,1);
//											
//										}
//										else ssd1306_WriteString(" ",font_8x14,1);
//								}
//								else
//								{	
//										ssd1306_SetCursor(0,15);
//										
//										//if (menu_485_points_for_showing != 0)	
//										{
//											strncpy(msg,"Значение регистра ", 18);
//											string_scroll_with_number(msg, 18, menu_485_points_for_showing);					

//											ssd1306_SetCursor(0,32);				
//											
//											if (master_array[menu_485_points_for_showing].master_type == 1 || master_array[menu_485_points_for_showing].master_type == 4)
//											{
//												snprintf(buffer, sizeof buffer, "%.02f", master_array[menu_485_points_for_showing].master_value);
//											}
//											else
//											{
//												snprintf(buffer, sizeof buffer, "%d", (int16_t) master_array[menu_485_points_for_showing].master_value);
//											}
//											
//											ssd1306_WriteString(buffer,font_8x14,1);											
//										}		

//								}
//								
//								//ssd1306_UpdateScreen();				
//							}

//							if (menu_edit_settings_mode == 0) 
//							for (uint8_t i = 0; i< REG_485_QTY; i++)
//							{								
//								
//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 0) //Значение регистра
//								{
//									ssd1306_Fill(0);
//									
//									ssd1306_SetCursor(0,0);																					
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);

//									triangle_left(55,0);									
//									if (i != REG_485_QTY-1) triangle_right(60,0);													
//									triangle_down(58,43);									
//									
//									ssd1306_SetCursor(0,15);										
//									strncpy(msg,"Значение регистра ", 18);						
//									string_scroll_with_number(msg, 18, i);

//									ssd1306_SetCursor(0,32);
//									if (master_array[i].master_type == 1 || master_array[i].master_type == 4)
//									{
//										snprintf(buffer, sizeof buffer, "%.02f", master_array[i].master_value);
//									}
//									else
//									{
//										snprintf(buffer, sizeof buffer, "%d", (int16_t) master_array[i].master_value);
//									}
//									ssd1306_WriteString(buffer,font_8x14,1);
//									
//									menu_edit_mode = 0 ; //Запрещаем редактирование												 									
//								}									
//																
//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 1) //Вкл/выкл опрос
//								{
//									ssd1306_Fill(0);
//									ssd1306_SetCursor(0,0);												
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);		

//									triangle_left(55,0);
//									if (i != REG_485_QTY-1) triangle_right(60,0);
//									triangle_up(58,38);
//									triangle_down(58,43);
//									
//									ssd1306_SetCursor(0,15);										
//									strncpy(msg,"Включить опрос регистра ", 24);						
//									string_scroll_with_number(msg, 24, i);
//									ssd1306_SetCursor(0,32);
//									
//									if (menu_edit_mode == 1) //Режим редактирования
//									{											
//											edit_mode_int(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 0]);																																													
//											//edit_mode_int8(&master_array[i].master_on);																																													
//									}
//									else //Нормальный режим
//									{
//										snprintf(buffer, sizeof buffer, "%d", master_array[i].master_on);
//										ssd1306_WriteString(buffer,font_8x14,1);										
//									}	
//									
//								}									
//								
//								
//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 2) //Адрес устройства
//								{
//									ssd1306_Fill(0);
//									ssd1306_SetCursor(0,0);												
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);	
//									
//									triangle_left(55,0);
//									if (i != REG_485_QTY-1) triangle_right(60,0);
//									triangle_up(58,38);
//									triangle_down(58,43);
//									
//									ssd1306_SetCursor(0,15);									
//									strncpy(msg,"Адрес устройства регистра ", 26);						
//									string_scroll_with_number(msg, 26, i);
//									
//									ssd1306_SetCursor(0,32);
//									
//									if (menu_edit_mode == 1) //Режим редактирования
//									{											
//											edit_mode_int(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 1]);																			
//									}
//									else //Нормальный режим
//									{
//										snprintf(buffer, sizeof buffer, "%d", master_array[i].master_addr);
//										ssd1306_WriteString(buffer,font_8x14,1);
//									}	
//								
//								}	
//								
//								
//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 3) //Номер регистра
//								{
//									ssd1306_Fill(0);
//									ssd1306_SetCursor(0,0);												
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);
//									
//									triangle_left(55,0);
//									if (i != REG_485_QTY-1) triangle_right(60,0);
//									triangle_up(58,38);
//									triangle_down(58,43);									
//									
//									ssd1306_SetCursor(0,15);										
//									strncpy(msg,"Адрес регистра ", 15);						
//									string_scroll_with_number(msg, 15, i);

//									ssd1306_SetCursor(0,32);									
//									if (menu_edit_mode == 1) //Режим редактирования
//									{											
//											edit_mode_int(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 2]);																			
//									}
//									else //Нормальный режим
//									{
//										snprintf(buffer, sizeof buffer, "%d", master_array[i].master_numreg);
//										ssd1306_WriteString(buffer,font_8x14,1);
//									}	
//						
//								}	

//								
//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 4) //Функциональный код
//								{
//									ssd1306_Fill(0);
//									ssd1306_SetCursor(0,0);												
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);

//									triangle_left(55,0);
//									if (i != REG_485_QTY-1) triangle_right(60,0);
//									triangle_up(58,38);
//									triangle_down(58,43);
//									
//									ssd1306_SetCursor(0,15);										
//									strncpy(msg,"Функциональный код регистра ", 28);						
//									string_scroll_with_number(msg, 28, i);

//									ssd1306_SetCursor(0,32);									
//									if (menu_edit_mode == 1) //Режим редактирования
//									{											
//											edit_mode_int(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 3]);																			
//									}
//									else //Нормальный режим
//									{
//										snprintf(buffer, sizeof buffer, "%d", master_array[i].master_func);
//										ssd1306_WriteString(buffer,font_8x14,1);
//									}	
//							
//								}
//								
//								
//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 5) //Нижняя предупредительная уставка
//								{
//									ssd1306_Fill(0);
//									ssd1306_SetCursor(0,0);												
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);

//									triangle_left(55,0);
//									if (i != REG_485_QTY-1) triangle_right(60,0);
//									triangle_up(58,38);
//									triangle_down(58,43);
//									
//									ssd1306_SetCursor(0,15);									
//									strncpy(msg,"Нижняя предупредительная уставка регистра ", 42);						
//									string_scroll_with_number(msg, 42, i);

//									ssd1306_SetCursor(0,32);									
//									if (menu_edit_mode == 1) //Режим редактирования
//									{											
//											edit_mode(&master_array[i].low_master_warning_set);
//										
//											convert_float_and_swap(master_array[i].low_master_warning_set, &temp_buf[0]);	 
//											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 12] = temp_buf[0];
//											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 13] = temp_buf[1];																														
//									}
//									else //Нормальный режим
//									{
//										snprintf(buffer, sizeof buffer, "%.01f", master_array[i].low_master_warning_set);
//										ssd1306_WriteString(buffer,font_8x14,1);
//									}	
//						
//								}
//								
//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 6) //Нижняя аварийная уставка
//								{
//									ssd1306_Fill(0);
//									ssd1306_SetCursor(0,0);												
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);

//									triangle_left(55,0);
//									if (i != REG_485_QTY-1) triangle_right(60,0);
//									triangle_up(58,38);
//									triangle_down(58,43);
//									
//									ssd1306_SetCursor(0,15);									
//									strncpy(msg,"Нижняя аварийная уставка регистра ", 34);						
//									string_scroll_with_number(msg, 34, i);

//									ssd1306_SetCursor(0,32);									
//									if (menu_edit_mode == 1) //Режим редактирования
//									{											
//											edit_mode(&master_array[i].low_master_emergency_set);
//										
//											convert_float_and_swap(master_array[i].low_master_emergency_set, &temp_buf[0]);	 
//											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 14] = temp_buf[0];
//											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 15] = temp_buf[1];																														
//									}
//									else //Нормальный режим
//									{
//										snprintf(buffer, sizeof buffer, "%.01f", master_array[i].low_master_emergency_set);
//										ssd1306_WriteString(buffer,font_8x14,1);
//									}									
//								}											


//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 7) //Верхняя предупредительная уставка
//								{
//									ssd1306_Fill(0);
//									ssd1306_SetCursor(0,0);												
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);

//									triangle_left(55,0);
//									if (i != REG_485_QTY-1) triangle_right(60,0);
//									triangle_up(58,38);
//									triangle_down(58,43);
//									
//									ssd1306_SetCursor(0,15);									
//									strncpy(msg,"Верхняя предупредительная уставка регистра ", 43);						
//									string_scroll_with_number(msg, 43, i);

//									ssd1306_SetCursor(0,32);									
//									if (menu_edit_mode == 1) //Режим редактирования
//									{											
//											edit_mode(&master_array[i].master_warning_set);
//										
//											convert_float_and_swap(master_array[i].master_warning_set, &temp_buf[0]);	 
//											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 16] = temp_buf[0];
//											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 17] = temp_buf[1];																														
//									}
//									else //Нормальный режим
//									{
//										snprintf(buffer, sizeof buffer, "%.01f", master_array[i].master_warning_set);
//										ssd1306_WriteString(buffer,font_8x14,1);
//									}	
//						
//								}		

//								
//								
//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 8) //Верхняя аварийная уставка
//								{
//									ssd1306_Fill(0);
//									ssd1306_SetCursor(0,0);												
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);

//									triangle_left(55,0);
//									if (i != REG_485_QTY-1) triangle_right(60,0);
//									triangle_up(58,38);
//									triangle_down(58,43);
//									
//									ssd1306_SetCursor(0,15);									
//									strncpy(msg,"Верхняя аварийная уставка регистра ", 35);						
//									string_scroll_with_number(msg, 35, i);

//									ssd1306_SetCursor(0,32);									
//									if (menu_edit_mode == 1) //Режим редактирования
//									{											
//											edit_mode(&master_array[i].master_emergency_set);
//										
//											convert_float_and_swap(master_array[i].master_emergency_set, &temp_buf[0]);	 
//											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 18] = temp_buf[0];
//											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 19] = temp_buf[1];																														
//									}
//									else //Нормальный режим
//									{
//										snprintf(buffer, sizeof buffer, "%.01f", master_array[i].master_emergency_set);
//										ssd1306_WriteString(buffer,font_8x14,1);
//									}									
//								}									

//								
//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 9) //Коэф. А
//								{
//									ssd1306_Fill(0);
//									ssd1306_SetCursor(0,0);												
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);
//									
//									triangle_left(55,0);
//									if (i != REG_485_QTY-1) triangle_right(60,0);
//									triangle_up(58,38);
//									triangle_down(58,43);
//									
//									ssd1306_SetCursor(0,15);										
//									strncpy(msg,"Коэффициент А регистра ", 23);						
//									string_scroll_with_number(msg, 23, i);

//									ssd1306_SetCursor(0,32);									
//									if (menu_edit_mode == 1) //Режим редактирования
//									{											
//											edit_mode(&master_array[i].master_coef_A);
//										
//											convert_float_and_swap(master_array[i].master_coef_A, &temp_buf[0]);	 
//											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 6] = temp_buf[0];
//											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 7] = temp_buf[1];																														
//									}
//									else //Нормальный режим
//									{
//										snprintf(buffer, sizeof buffer, "%.05f", master_array[i].master_coef_A);
//										ssd1306_WriteString(buffer,font_8x14,1);
//									}									
//								}		

//								
//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 10) //Коэф. B
//								{
//									ssd1306_Fill(0);
//									ssd1306_SetCursor(0,0);												
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);

//									triangle_left(55,0);
//									if (i != REG_485_QTY-1) triangle_right(60,0);
//									triangle_up(58,38);
//									triangle_down(58,43);
//									
//									ssd1306_SetCursor(0,15);										
//									strncpy(msg,"Коэффициент В регистра ", 23);						
//									string_scroll_with_number(msg, 23, i);

//									ssd1306_SetCursor(0,32);									
//									if (menu_edit_mode == 1) //Режим редактирования
//									{											
//											edit_mode(&master_array[i].master_coef_B);
//										
//											convert_float_and_swap(master_array[i].master_coef_B, &temp_buf[0]);	 
//											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 8] = temp_buf[0];
//											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 9] = temp_buf[1];																														
//									}
//									else //Нормальный режим
//									{
//										snprintf(buffer, sizeof buffer, "%.05f", master_array[i].master_coef_B);
//										ssd1306_WriteString(buffer,font_8x14,1);
//									}																
//								}	

//								
//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 11) //Тип данных
//								{
//									ssd1306_Fill(0);
//									ssd1306_SetCursor(0,0);												
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);

//									triangle_left(55,0);
//									if (i != REG_485_QTY-1) triangle_right(60,0);
//									triangle_up(58,38);
//									triangle_down(58,43);
//									
//									ssd1306_SetCursor(0,15);										
//									strncpy(msg,"Тип данных регистра ", 20);						
//									string_scroll_with_number(msg, 20, i);

//									ssd1306_SetCursor(0,32);									
//									if (menu_edit_mode == 1) //Режим редактирования
//									{											
//											edit_mode_int(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 4]);										
//									}
//									else //Нормальный режим
//									{
//											snprintf(buffer, sizeof buffer, "%d", master_array[i].master_type);
//											ssd1306_WriteString(buffer,font_8x14,1);
//									}																						
//								}				

//								
//								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 12) //Таймаут
//								{
//									ssd1306_Fill(0);
//									ssd1306_SetCursor(0,0);												
//									snprintf(buffer, sizeof buffer, "485 %d", i);
//									ssd1306_WriteString(buffer,font_8x14,1);
//									
//									triangle_left(55,0);
//									if (i != REG_485_QTY-1) triangle_right(60,0);
//									triangle_up(58,38);
//																		
//									ssd1306_SetCursor(0,15);										
//									strncpy(msg,"Таймаут регистра ", 17);						
//									string_scroll_with_number(msg, 17, i);

//									ssd1306_SetCursor(0,32);									
//									if (menu_edit_mode == 1) //Режим редактирования
//									{											
//											edit_mode_int(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 5]);										
//									}
//									else //Нормальный режим
//									{
//											snprintf(buffer, sizeof buffer, "%d", master_array[i].request_timeout);
//											ssd1306_WriteString(buffer,font_8x14,1);
//									}																						
//								}									
//								
//							}							
//							
//								
//							if (menu_index_pointer == 3 && menu_horizontal == 1 && menu_edit_settings_mode == 1) //Номер параметра для показа на гл. экране
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("485",font_8x14,1);												
//								triangle_left(50,0);						
//								triangle_right(55,0);							
//								triangle_right(60,0);		
//								
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Параметр на главном меню", 24);						
//								string_scroll(msg, 24);
//								
//								ssd1306_SetCursor(0,32);			
//								
//								if (menu_edit_mode == 1) //Режим редактирования
//								{											
//											edit_mode_int(&menu_485_points_for_showing);										
//								}
//								else //Нормальный режим
//								{
//									snprintf(buffer, sizeof buffer, "%d", menu_485_points_for_showing);
//									ssd1306_WriteString(buffer,font_8x14,1); 
//								}										
//												
//							}
//							
//							if (menu_index_pointer == 3 && menu_horizontal == 2 && menu_edit_settings_mode == 1) //Скорость обмена
//							{
//								ssd1306_Fill(0);
//								ssd1306_SetCursor(0,0);												
//								ssd1306_WriteString("485",font_8x14,1);												
//								triangle_left(50,0);														
//																
//								ssd1306_SetCursor(0,15);									
//								strncpy(msg,"Скорость", 8);						
//								string_scroll(msg, 8);
//								
//								ssd1306_SetCursor(0,32);			
//								
//								if (menu_edit_mode == 1) //Режим редактирования
//								{
//									edit_mode_from_list(&baud_rate_uart_3, (uint32_t*)&baudrate_array);
//								}
//								else 
//								{
//									snprintf(buffer, sizeof buffer, "%.00f", baud_rate_uart_3);			
//									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//								}								
//												
//							}								
//							
//							//ssd1306_UpdateScreen();
//					}
//					
////////////Реле

//					if (menu_index_pointer == 4 && menu_horizontal == 0) //Состояние реле
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Реле",font_8x15_RU,1);																
//						
//						if (menu_edit_settings_mode == 0) 
//						{
//							triangle_right(60,0);																								
//							triangle_up(58,38);
//							triangle_down(58,43);							
//						}
//						else
//						{
//							triangle_right(55,0);
//							triangle_right(60,0);																								
//							triangle_up(58,38);
//							triangle_down(58,43);
//						}
//						
//						ssd1306_SetCursor(0,15);																									
//						ssd1306_WriteString("Пред",font_8x15_RU,1);		
//						ssd1306_WriteString(".",font_8x14,1);		
//						ssd1306_SetCursor(42,16);
//						snprintf(buffer, sizeof buffer, "%d", state_warning_relay);
//						ssd1306_WriteString(buffer,font_8x14,1);							
//						ssd1306_SetCursor(0,32);				
//						ssd1306_WriteString("Авар",font_8x15_RU,1);		
//						ssd1306_WriteString(".",font_8x14,1);		
//						ssd1306_SetCursor(42,33);
//						snprintf(buffer, sizeof buffer, "%d", state_emerg_relay);
//						ssd1306_WriteString(buffer,font_8x14,1);							
//						
//						//ssd1306_UpdateScreen();				
//						
//						disable_up_down_button = 0;
//					}							
//					
//					
//					if (menu_index_pointer == 4 && menu_horizontal == 1 && menu_edit_settings_mode == 0) //Аттрибут события ICP, 4-20
//					{
//						ssd1306_Fill(0);												
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Реле",font_8x15_RU,1);									
//						
//						triangle_left(55,0);
//						triangle_right(60,0);																								
//						//triangle_up(58,38);
//						//triangle_down(58,43);
//						
//						ssd1306_SetCursor(0,15);						
//						strncpy(msg,"Аттрибут события", 16);						
//						string_scroll(msg, 16);
//						ssd1306_WriteString(" 1",font_8x14,1);
//						
//						ssd1306_SetCursor(0,32);														
//						snprintf(buffer, sizeof buffer, "0x%X", trigger_event_attribute);			
//						ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим

//						menu_edit_mode = 0 ; //Запрещаем редактирование						
//						
//						ssd1306_UpdateScreen();			

//						disable_up_down_button = 1;
//					}						
//					
//					if (menu_index_pointer == 4 && menu_horizontal == 2 && menu_edit_settings_mode == 0) //Аттрибут события 485 пред. уставка
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Реле",font_8x15_RU,1);			

//						triangle_left(55,0);
//						triangle_right(60,0);																								
//						//triangle_up(58,38);
//						//triangle_down(58,43);
//						
//						ssd1306_SetCursor(0,15);	
//						
//						strncpy(msg,"Аттрибут события", 16);						
//						string_scroll(msg, 16);
//						ssd1306_WriteString(" 2",font_8x14,1);
//						
//						ssd1306_SetCursor(0,32);														
//						snprintf(buffer, sizeof buffer, "0x%X", trigger_485_event_attribute_warning);			
//						ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим

//						menu_edit_mode = 0 ; //Запрещаем редактирование
//						
//						//ssd1306_UpdateScreen();		

//						disable_up_down_button = 1;
//					}						
//					
//					if (menu_index_pointer == 4 && menu_horizontal == 3 && menu_edit_settings_mode == 0) //Аттрибут события 485 авар. уставка
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Реле",font_8x15_RU,1);			
//						
//						triangle_left(55,0);																														
//						//triangle_up(58,38);
//						//triangle_down(58,43);
//						
//						ssd1306_SetCursor(0,15);						
//						strncpy(msg,"Аттрибут события", 16);						
//						string_scroll(msg, 16);
//						ssd1306_WriteString(" 3",font_8x14,1);
//						
//						ssd1306_SetCursor(0,32);														
//						snprintf(buffer, sizeof buffer, "0x%X", trigger_485_event_attribute_emerg);			
//						ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим

//						menu_edit_mode = 0 ; //Запрещаем редактирование
//						
//						//ssd1306_UpdateScreen();				
//						
//						disable_up_down_button = 1;
//					}							
//					
//					//Режим редактирования настроек реле
//					if (menu_index_pointer == 4 && menu_horizontal == 1 && menu_edit_settings_mode == 1) //Режим работы реле
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Реле",font_8x15_RU,1);			

//						triangle_left(50,0);
//						triangle_right(55,0);
//						triangle_right(60,0);
//						//triangle_up(58,38);
//						//triangle_down(58,43);
//						
//						ssd1306_SetCursor(0,15);	
//						ssd1306_WriteString("Режим",font_8x15_RU,1);		
//						
//						osDelay(200);
//												
//						ssd1306_SetCursor(0,32);						
//						if (menu_edit_mode == 1) //Режим редактирования
//						{
//							edit_mode_int(&mode_relay);
//							disable_up_down_button = 0;
//						}
//						else 
//						{
//							snprintf(buffer, sizeof buffer, "%d", mode_relay);			
//							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//							disable_up_down_button = 1;
//						}
//												
//						//ssd1306_UpdateScreen();				
//					}						


//					if (menu_index_pointer == 4 && menu_horizontal == 2 && menu_edit_settings_mode == 1) //Задержка на срабатывание реле
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Реле",font_8x15_RU,1);			

//						triangle_left(50,0);
//						triangle_right(55,0);
//						triangle_right(60,0);
//						//triangle_up(58,38);
//						//triangle_down(58,43);						
//						
//						ssd1306_SetCursor(0,15);					
//						strncpy(msg,"Задержка на срабатывание", 24);						
//						string_scroll(msg, 24);
//						
//						ssd1306_SetCursor(0,32);						
//						if (menu_edit_mode == 1) //Режим редактирования
//						{
//							edit_mode_int(&delay_relay);
//							disable_up_down_button = 0;
//						}
//						else 
//						{
//							snprintf(buffer, sizeof buffer, "%d", delay_relay);			
//							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//							disable_up_down_button = 1;
//						}
//												
//						//ssd1306_UpdateScreen();				
//					}						
//					
//					
//					if (menu_index_pointer == 4 && menu_horizontal == 3 && menu_edit_settings_mode == 1) //Задержка на выход из срабатывания реле
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Реле",font_8x15_RU,1);			

//						triangle_left(50,0);
//						triangle_right(55,0);
//						triangle_right(60,0);
//						//triangle_up(58,38);
//						//triangle_down(58,43);
//						
//						ssd1306_SetCursor(0,15);
//						strncpy(msg,"Задержка на выход из срабатывания", 33);						
//						string_scroll(msg, 33);
//						
//						ssd1306_SetCursor(0,32);						
//						if (menu_edit_mode == 1) //Режим редактирования
//						{
//							edit_mode_int(&delay_relay_exit);
//							disable_up_down_button = 0;
//						}
//						else 
//						{
//							snprintf(buffer, sizeof buffer, "%d", delay_relay_exit);			
//							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//							disable_up_down_button = 1;
//						}
//			
//												
//						//ssd1306_UpdateScreen();				
//					}	
//					

//					if (menu_index_pointer == 4 && menu_horizontal == 4 && menu_edit_settings_mode == 1) //Тест работы реле
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Реле",font_8x15_RU,1);			
//						
//						triangle_left(50,0);						
//						//triangle_up(58,38);
//						//triangle_down(58,43);
//						
//						ssd1306_SetCursor(0,15);	
//						strncpy(msg,"Тест реле", 9);						
//						string_scroll(msg, 9);
//						
//						ssd1306_SetCursor(0,32);																
//						if (menu_edit_mode == 1) //Режим редактирования
//						{
//							edit_mode_int(&test_relay);
//							disable_up_down_button = 0;
//						}
//						else 
//						{
//							snprintf(buffer, sizeof buffer, "%d", test_relay);			
//							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//							disable_up_down_button = 1;
//						}	
//												
//						//ssd1306_UpdateScreen();				
//					}						
//					
//					
//					
//					
////////////Общие настройки	
//					
//					if (menu_index_pointer == 5 && menu_horizontal == 0) 
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//										
//						triangle_right(60,0);
//						triangle_up(58,38);
//						triangle_down(58,43);						
//						
//						ssd1306_SetCursor(0,15);																									
//						strncpy(msg,"Настройки", 9);						
//						string_scroll(msg, 9);						

//						//horizont_line(0,45);						
//						
//						//ssd1306_UpdateScreen();				
//						
//						disable_up_down_button = 0;
//					}
//					
//					
//					if (menu_index_pointer == 5 && menu_horizontal == 1) //Адрес устройства
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Настр",font_8x15_RU,1);																
//						ssd1306_WriteString(".",font_8x14,1);					
//			
//						triangle_left(55,0);
//						triangle_right(60,0);
//						//triangle_up(58,38);
//						//triangle_down(58,43);						
//						
//						ssd1306_SetCursor(0,15);	
//						ssd1306_WriteString("Адрес",font_8x15_RU,1);		
//						osDelay(200);
//						ssd1306_SetCursor(0,32);						
//						if (menu_edit_mode == 1) //Режим редактирования
//						{
//							edit_mode_int(&slave_adr);
//							disable_up_down_button = 0;
//						}
//						else 
//						{
//							snprintf(buffer, sizeof buffer, "%d", slave_adr);			
//							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//							disable_up_down_button = 1;
//						}
//												
//						//ssd1306_UpdateScreen();				
//					}	
//												
//					if (menu_index_pointer == 5 && menu_horizontal == 2) //Скорость обмена
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Настр",font_8x15_RU,1);																
//						ssd1306_WriteString(".",font_8x14,1);					

//						triangle_left(55,0);
//						triangle_right(60,0);
//						//triangle_up(58,38);
//						//triangle_down(58,43);
//						
//						ssd1306_SetCursor(0,15);
//						strncpy(msg,"Скорость", 8);						
//						string_scroll(msg, 8);
//						
//						ssd1306_SetCursor(0,32);						
//						if (menu_edit_mode == 1) //Режим редактирования
//						{
//							edit_mode_from_list(&baud_rate_uart_2, (uint32_t*)&baudrate_array);
//							disable_up_down_button = 0;
//						}
//						else 
//						{
//							snprintf(buffer, sizeof buffer, "%.00f", baud_rate_uart_2);			
//							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//							disable_up_down_button = 1;
//						}
//												
//						//ssd1306_UpdateScreen();				
//					}						
//					
//									
//					
//					if (menu_index_pointer == 5 && menu_horizontal == 3) //Время прогрева
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Настр",font_8x15_RU,1);																
//						ssd1306_WriteString(".",font_8x14,1);					
//						
//						triangle_left(55,0);
//						triangle_right(60,0);
//						//triangle_up(58,38);
//						//triangle_down(58,43);
//						
//						ssd1306_SetCursor(0,15);
//						strncpy(msg,"Время прогрева", 14);						
//						string_scroll(msg, 14);
//						
//						ssd1306_SetCursor(0,32);						
//						if (menu_edit_mode == 1) //Режим редактирования
//						{
//							edit_mode_int(&warming_up);
//							disable_up_down_button = 0;
//						}
//						else 
//						{
//							snprintf(buffer, sizeof buffer, "%d", warming_up);			
//							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//							disable_up_down_button = 1;
//						}
//												
//						//ssd1306_UpdateScreen();				
//					}		
//					
//					if (menu_index_pointer == 5 && menu_horizontal == 4) //Сброс настроек 
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Настр",font_8x15_RU,1);																
//						ssd1306_WriteString(".",font_8x14,1);					
//						
//						triangle_left(55,0);						
//						//triangle_up(58,38);
//						//triangle_down(58,43);
//						
//						ssd1306_SetCursor(0,15);
//						strncpy(msg,"Сброс настроек", 14);						
//						string_scroll(msg, 14);
//						
//						ssd1306_SetCursor(0,32);						
//						if (menu_edit_mode == 1) //Режим редактирования
//						{
//							edit_mode_int(&reset_to_default);
//							disable_up_down_button = 0;
//						}
//						else 
//						{
//							snprintf(buffer, sizeof buffer, "%d", reset_to_default);			
//							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//							disable_up_down_button = 1;
//						}

//						//ssd1306_UpdateScreen();						
//					}						



//////////////Информация

//					if (menu_index_pointer == 6 && menu_horizontal == 0) 
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);																		
//												
//						triangle_right(60,0);
//						triangle_up(58,38);
//												
//						ssd1306_SetCursor(0,15);											
//						strncpy(msg,"Информация", 10);						
//						string_scroll(msg, 10);							
//						//ssd1306_UpdateScreen();				
//						disable_up_down_button = 0;
//					}

//					
//					if (menu_index_pointer == 6 && menu_horizontal == 1) //Напряжение питания контроллера
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Инф",font_8x15_RU,1);																
//						ssd1306_WriteString(".",font_8x14,1);					
//						
//						triangle_left(55,0);
//						triangle_right(60,0);
//						//triangle_up(58,38);
//						
//						ssd1306_SetCursor(0,15);
//						strncpy(msg,"Напряжение питания", 18);						
//						string_scroll(msg, 18);
//						
//						ssd1306_SetCursor(0,32);				
//						snprintf(buffer, sizeof buffer, "%.01f", power_supply_voltage);				
//						ssd1306_WriteString(buffer,font_8x14,1);
//						//ssd1306_UpdateScreen();				
//						
//						disable_up_down_button = 1;
//					}	
//					
//					if (menu_index_pointer == 6 && menu_horizontal == 2) //Версия ПО
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Инф",font_8x15_RU,1);																
//						ssd1306_WriteString(".",font_8x14,1);					

//						triangle_left(55,0);
//						triangle_right(60,0);
//						//triangle_up(58,38);						
//						
//						ssd1306_SetCursor(0,15);
//						strncpy(msg,"Версия ПО", 9);						
//						string_scroll(msg, 9);
//						
//						ssd1306_SetCursor(0,32);				
//						snprintf(buffer, sizeof buffer, "%.02f", VERSION);				
//						ssd1306_WriteString(buffer,font_8x14,1);
//						//ssd1306_UpdateScreen();			

//						disable_up_down_button = 1;						
//					}
//					
//					
//					if (menu_index_pointer == 6 && menu_horizontal == 3) //% ошибок timeout modbus master
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Инф",font_8x15_RU,1);																
//						ssd1306_WriteString(".",font_8x14,1);					
//						
//						triangle_left(55,0);						
//						//triangle_up(58,38);						
//						
//						ssd1306_SetCursor(0,15);
//						ssd1306_WriteString("MMTE",font_8x14,1);	
//						
//						ssd1306_SetCursor(0,32);				
//						snprintf(buffer, sizeof buffer, "%.01f", mb_master_timeout_error_percent);				
//						ssd1306_WriteString(buffer,font_8x14,1);
//						//ssd1306_UpdateScreen();				
//						
//						disable_up_down_button = 1;
//					}					

//							
//					

////////////Конфигурация
//					
//					if (menu_index_pointer == 7 && menu_horizontal == 0 && config_mode == 1) 
//					{
//						ssd1306_Fill(0);											
//						triangle_right(55,2);						
//						ssd1306_SetCursor(0,15);																									

//						strncpy(msg,"КОНФИГУРАЦИЯ", 12);						
//						string_scroll(msg, 12);						

//						//ssd1306_UpdateScreen();				
//					}
//					
//					if (menu_index_pointer == 7 && menu_horizontal == 1) //Включаем канал ICP
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Конф",font_8x15_RU,1);																
//						ssd1306_WriteString(".",font_8x14,1);					
//						triangle_left(48,2);						
//						triangle_right(55,2);				
//						ssd1306_SetCursor(0,15);	
//						ssd1306_WriteString("ICP",font_8x14,1);		
//						ssd1306_SetCursor(0,32);				
//						
//						
//						if (menu_edit_mode == 1) //Режим редактирования
//						{
//							edit_mode_int(&channel_ICP_ON);
//						}
//						else 
//						{
//							snprintf(buffer, sizeof buffer, "%d", channel_ICP_ON);			
//							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//						}
//												
//						//ssd1306_UpdateScreen();				
//					}	
//					
//					
//					if (menu_index_pointer == 7 && menu_horizontal == 2) //Включаем канал 4-20
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Конф",font_8x15_RU,1);																
//						ssd1306_WriteString(".",font_8x14,1);					
//						triangle_left(48,2);						
//						triangle_right(55,2);				
//						ssd1306_SetCursor(0,15);	
//						ssd1306_WriteString("4-20",font_8x14,1);		
//						ssd1306_SetCursor(0,32);				
//						
//						
//						if (menu_edit_mode == 1) //Режим редактирования
//						{
//							edit_mode_int(&channel_4_20_ON);
//						}
//						else 
//						{
//							snprintf(buffer, sizeof buffer, "%d", channel_4_20_ON);			
//							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//						}
//												
//						//ssd1306_UpdateScreen();				
//					}	
//					
//					
//					if (menu_index_pointer == 7 && menu_horizontal == 3) //Включаем канал 485
//					{
//						ssd1306_Fill(0);
//						ssd1306_SetCursor(0,0);												
//						ssd1306_WriteString("Конф",font_8x15_RU,1);																
//						ssd1306_WriteString(".",font_8x14,1);					
//						triangle_left(48,2);														
//						ssd1306_SetCursor(0,15);	
//						ssd1306_WriteString("485",font_8x14,1);		
//						ssd1306_SetCursor(0,32);				
//						
//						
//						if (menu_edit_mode == 1) //Режим редактирования
//						{
//							edit_mode_int(&channel_485_ON);
//						}
//						else 
//						{
//							snprintf(buffer, sizeof buffer, "%d", channel_485_ON);			
//							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
//						}
//												
//						//ssd1306_UpdateScreen();				
//					}	
//					
//					
//				//Рисуем на экранчике
//				ssd1306_UpdateScreen();	
//					
//				//Инверсия переменной (для мигания меню в режиме редакции)	
//				temp_stat_1 = !temp_stat_1;

//			
//			}
	
			osDelay(100);
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



//uint32_t rtc_read_backup_reg(uint32_t BackupRegister) 
//{
//    RTC_HandleTypeDef RtcHandle;
//    RtcHandle.Instance = RTC;
//    return HAL_RTCEx_BKUPRead(&RtcHandle, BackupRegister);
//}
// 
//void rtc_write_backup_reg(uint32_t BackupRegister, uint32_t data) 
//{
//    RTC_HandleTypeDef RtcHandle;
//    RtcHandle.Instance = RTC;
//    HAL_PWR_EnableBkUpAccess();
//    HAL_RTCEx_BKUPWrite(&RtcHandle, BackupRegister, data);
//    HAL_PWR_DisableBkUpAccess();
//}

void string_scroll(char* msg, uint8_t len)
{		
	
	for(int i = temp_str; i < len; i++)
	{	
		ssd1306_WriteChar(msg[i],font_8x15_RU,1);				
	}
	
	if ( temp_str > len ) 
	{
		temp_str = 0;				
	}
	else 
	{
		temp_str++;		
	}
	
	
	osDelay(200);
	
}

void string_scroll_with_number(char* msg, uint8_t len, uint8_t number)
{		
	
	for(int i = temp_str; i < len; i++)
	{	
		ssd1306_WriteChar(msg[i],font_8x15_RU,1);		
	}
	
	snprintf(buffer, sizeof buffer, "%d", number);
	ssd1306_WriteChar(buffer[0],font_8x14,1);		
	
	if ( temp_str > len ) 
	{
		temp_str = 0;		
	}
	else 
	{
		temp_str++;		
	}
	
	osDelay(200);
	
}

void edit_mode(float32_t *var)
{
	//Целая часть
	if (temp_stat_1 == 0 && digit_rank == 0) 
	{									
		snprintf(buffer, sizeof buffer, "%.01f", *var);									
		ssd1306_WriteString(buffer,font_8x14,1);			
	}
	
//	if (temp_stat_1 == 1 && digit_rank == 0) 
//	{									
//		snprintf(buffer, sizeof buffer, "%.00f", *var - (int16_t)(*var));									
//		ssd1306_WriteString(buffer,font_8x14,1);			
//	}	
	

	//Дробная часть
	if (temp_stat_1 == 0 && digit_rank == 1) 
	{
		snprintf(buffer, sizeof buffer, "%.01f", *var);									
		ssd1306_WriteString(buffer,font_8x14,1);
	}	
		
	if (temp_stat_1 == 1 && digit_rank == 1) 
	{
		if (*var < 0 && *var > -1) snprintf(buffer, sizeof buffer, "-%d", 0);									
		else snprintf(buffer, sizeof buffer, "%d", (int16_t) *var);									
		ssd1306_WriteString(buffer,font_8x14,1);
	}
						
	
	//Изменяем значение
	if (button_up_pressed_in == 1 && digit_rank == 0) 
	{ 
			*var+=1.0; 
			button_up_pressed_in = 0; 
	};
	
	if (button_up_pressed_in == 1 && digit_rank == 1) 
	{ 
			*var+=0.1; 
			button_up_pressed_in = 0; 
	};
	
	if (button_down_pressed_in == 1 && digit_rank == 0) 
	{ 
			*var-=1.0;  
			button_down_pressed_in = 0; 
	};
	
	if (button_down_pressed_in == 1 && digit_rank == 1) 
	{ 
			*var-=0.1; 
			button_down_pressed_in = 0; 
	};
	
	//Округляем до сотых для корректного отображения в меню
	*var = roundf(*var * 100) / 100;
}	

void edit_mode_int8(uint8_t *var) 
{

	if (temp_stat_1 == 0 && digit_rank == 0) 
	{									
		snprintf(buffer, sizeof buffer, "%d", *var);									
		ssd1306_WriteString(buffer,font_8x14,1);
	}
	else if (temp_stat_1 == 1 && digit_rank == 0) 
	{		
		snprintf(buffer, sizeof buffer, "", *var);									
		ssd1306_WriteString(buffer,font_8x14, 0);								
	}															
	
	//Изменяем значение
	if (button_up_pressed_in == 1 && digit_rank == 0) 
	{ 
			*var+=1; 
			button_up_pressed_in = 0; 
	};

	if (button_down_pressed_in == 1 && digit_rank == 0) 
	{ 
			*var-=1;  
			button_down_pressed_in = 0; 
	};

}	

void edit_mode_int(int16_t *var) 
{

	if (temp_stat_1 == 0 && digit_rank == 0) 
	{									
		snprintf(buffer, sizeof buffer, "%d", *var);									
		ssd1306_WriteString(buffer,font_8x14,1);
	}
	else if (temp_stat_1 == 1 && digit_rank == 0) 
	{		
		snprintf(buffer, sizeof buffer, "", *var);									
		ssd1306_WriteString(buffer,font_8x14, 0);								
	}															
	
	//Изменяем значение
	if (button_up_pressed_in == 1 && digit_rank == 0) 
	{ 
			*var+=1; 
			button_up_pressed_in = 0; 
	};

	if (button_down_pressed_in == 1 && digit_rank == 0) 
	{ 
			*var-=1;  
			button_down_pressed_in = 0; 
	};

}	

void edit_mode_from_list(float32_t *var, uint32_t* list)
{
	
	//Целая часть
	if (temp_stat_1 == 0) 
	{									
		snprintf(buffer, sizeof buffer, "%d", (int)*var);									
		ssd1306_WriteString(buffer,font_8x14,1);
	}
	else if (temp_stat_1 == 1) 
	{
		fractpart = modf(*var, &intpart)*10;
		snprintf(buffer, sizeof buffer, "%d", (int)*var);									
		ssd1306_WriteString(buffer,font_8x14, 0);								
	}															
		
	//Изменяем значение
	if (button_up_pressed_in == 1) 
	{ 
			if (iter < 15) *var=list[iter++];					
			button_up_pressed_in = 0; 
	};

	
	if (button_down_pressed_in == 1) 
	{ 
			if (iter > 0) *var=list[iter--];  
			button_down_pressed_in = 0; 
	};
	

}	

void init_menu(uint8_t where_from) //Иниц. меню (1 - конфиг, т.е. зажата кнопка при вкл.)
{	
	number_of_items_in_the_menu = 0;
	menu_horizontal = 0;
	config_mode = 0;
	menu_index = 0;
	
	if (channel_ICP_ON == 1) 
	{			
		menu_index_array[number_of_items_in_the_menu] = items_menu_icp;
		number_of_items_in_the_menu++;
	}
	
	if (channel_4_20_ON == 1) 
	{
		menu_index_array[number_of_items_in_the_menu] = items_menu_4_20;
		number_of_items_in_the_menu++;
	}
	
	if (channel_485_ON == 1)
	{
		menu_index_array[number_of_items_in_the_menu] = items_menu_485;
		number_of_items_in_the_menu++;
	}
	
	menu_index_array[number_of_items_in_the_menu] = items_menu_relay;
	number_of_items_in_the_menu ++; //Реле
	
	menu_index_array[number_of_items_in_the_menu] = items_menu_common;
	number_of_items_in_the_menu ++; //Основные настройки
 			
	menu_index_array[number_of_items_in_the_menu] = items_menu_info;
	number_of_items_in_the_menu ++; //Информация
	
	
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0 && where_from == 1) 
	{
		config_mode = 1;	
		menu_index_array[number_of_items_in_the_menu] = items_menu_config;
		number_of_items_in_the_menu++; //Включаем доп. меню для конфигурации								
	}	
	
	menu_index_pointer = menu_index_array[0];	

}

void save_settings(void)
{
			uint16_t temp[2];	
			volatile uint8_t res = 0;
	
			//xSemaphoreTake( Mutex_Setting, portMAX_DELAY );
	
	
			convert_float_and_swap(hi_warning_icp, &temp[0]);		
			settings[4] = temp[0];
			settings[5] = temp[1];
			convert_float_and_swap(hi_emerg_icp, &temp[0]);		
			settings[8] = temp[0];
			settings[9] = temp[1];		
			settings[19] = filter_mode_icp;	
	
			settings[29] = icp_menu_points_for_showing;		
	
			
			convert_float_and_swap(lo_warning_420, &temp[0]);		
			settings[38] = temp[0];
			settings[39] = temp[1];	
			convert_float_and_swap(lo_emerg_420, &temp[0]);		
			settings[42] = temp[0];
			settings[43] = temp[1];	
			convert_float_and_swap(hi_warning_420, &temp[0]);		
			settings[40] = temp[0];
			settings[41] = temp[1];	
			convert_float_and_swap(hi_emerg_420, &temp[0]);		
			settings[44] = temp[0];
			settings[45] = temp[1];				
			convert_float_and_swap(down_user_range_4_20, &temp[0]);		
			settings[47] = temp[0];
			settings[48] = temp[1];	
			convert_float_and_swap(up_user_range_4_20, &temp[0]);		
			settings[49] = temp[0];
			settings[50] = temp[1];	
			
			
			//settings[64] = slave_adr_mb_master;				
			convert_float_and_swap(baud_rate_uart_3, &temp[0]);
			settings[65] = temp[0];
			settings[66] = temp[1];										
			//settings[68] = slave_reg_mb_master;					
			//settings[70] = slave_func_mb_master;
			//settings[71] = quantity_reg_mb_master;
			
			settings[84] = mode_relay;
			settings[86] = delay_relay;
			settings[88] = delay_relay_exit;
			
			settings[100] = slave_adr;
			convert_float_and_swap(baud_rate_uart_2, &temp[0]);
			settings[101] = temp[0];
			settings[102] = temp[1];										
			settings[109] = warming_up;
			
			settings[28] = channel_ICP_ON;
			settings[57] = channel_4_20_ON;
			settings[72] = channel_485_ON;
		
				
				
			taskENTER_CRITICAL(); 									
			res = write_registers_to_flash(settings);				
			taskEXIT_CRITICAL(); 			
		

			init_menu(0);			
	
			ssd1306_Fill(0);
			ssd1306_SetCursor(0,0);												
			ssd1306_WriteString("Настр",font_8x15_RU,1);																												
			ssd1306_WriteString(".",font_8x14,1);	
			ssd1306_SetCursor(0,15);	
			ssd1306_WriteString("сохр",font_8x15_RU,1);																																		
			ssd1306_WriteString(".",font_8x14,1);	
			ssd1306_UpdateScreen();			
			osDelay(2000);	
	
			//xSemaphoreGive( Mutex_Setting );
			
			//NVIC_SystemReset();		
}

void turnover_counter(float32_t* input_array)
{	

	uint32_t pindex = 0;

		
	arm_mean_f32(&input_array[0], ADC_BUFFER_SIZE, &mean_level_TOC);
	level_summa_TOC += mean_level_TOC;

		
	for (uint16_t i=0; i<ADC_BUFFER_SIZE; i++)
	{
		
		if (impulse_sign == 1)
		{		
				if ( input_array[i] > common_level_TOC + hysteresis_TOC) 
				{						
					turnover_front = 1;
				}		
				else 
				{			
					turnover_front = 0;
				}		
			
				//Детектор переднего фронта	(+)
				if (turnover_front == 1 && old_turnover_front == 0) 
				{			
					difference_points_counter = big_points_counter - small_points_counter;
					
					turnover_summa[summa_iter++] = difference_points_counter;
					
					if (summa_iter == TOC_QUEUE_LENGHT) summa_iter = 0;			
					
					small_points_counter = big_points_counter;					
				}		
		}
		else
		{

				if ( input_array[i] < common_level_TOC - hysteresis_TOC) 
				{						
					turnover_front = 1;
				}		
				else 
				{			
					turnover_front = 0;
				}		
			
				//Детектор переднего фронта	(-)
				if (turnover_front == 1 && old_turnover_front == 0) 
				{			
					difference_points_counter = big_points_counter - small_points_counter;
					
					turnover_summa[summa_iter++] = difference_points_counter;
					
					if (summa_iter == TOC_QUEUE_LENGHT) summa_iter = 0;			
					
					small_points_counter = big_points_counter;					
				}				
		}
		
		
		old_turnover_front = turnover_front;
		
		if (big_points_counter >= 18446744073709551615) big_points_counter = 0;
		else big_points_counter++;		
		
	}

	//1 секунда
	if (pass_count == TOC_QUEUE_LENGHT )
	{	
		arm_mean_f32(&turnover_summa[0], TOC_QUEUE_LENGHT, &turnover_count_1s);
				
		turnover_count_60s = (float32_t) (25600.0 / (float32_t) turnover_count_1s) * 60.0;
		 		
		//xQueueSend(queue_TOC, (void*)&turnover_count_1s, 0);			
				
		common_level_TOC = level_summa_TOC / TOC_QUEUE_LENGHT;		
		level_summa_TOC = 0.0;				
		
		pass_count = 0;
	}	
	else pass_count++;
	
	
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
