/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "arm_math.h"
#include "math.h"
#include <stdint.h>
#include "Task_manager.h"
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "dac.h"
#include "fonts.h"
#include "ssd1306.h"

#include "stm32l4xx_it.h"
#include "modbus_reg_map.h"
#include "flash_manager.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

xSemaphoreHandle 	Semaphore1, Semaphore2,
									Semaphore_Acceleration, Semaphore_Velocity, Semaphore_Displacement,
									Q_Semaphore_Acceleration, Q_Semaphore_Velocity, Q_Semaphore_Displacement,
									Semaphore_Modbus_Rx, Semaphore_Modbus_Tx, 
									Semaphore_Master_Modbus_Rx, Semaphore_Master_Modbus_Tx,
									Semaphore_Relay_1, Semaphore_Relay_2,
									Semaphore_HART_Receive, Semaphore_HART_Transmit,
									Mutex_Setting, Mutex_Average,
									Semaphore_TBUS_Modbus_Rx, Semaphore_TBUS_Modbus_Tx, 
									Semaphore_Bootloader_Update, Semaphore_Bootloader_Erase;
									
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
int16_t mirror_values[MIRROR_COUNT]; //зеркало

uint8_t button_state = 0;

uint8_t transmitBuffer[REG_COUNT*2+5];
uint8_t receiveBuffer[256];
uint8_t boot_receiveBuffer[128];
uint8_t master_transmitBuffer[8];
uint8_t master_receiveBuffer[256];
uint8_t HART_receiveBuffer[16];
uint8_t HART_transmitBuffer[8];
uint8_t TBUS_transmitBuffer[REG_COUNT*2+5];
uint8_t TBUS_receiveBuffer[256];

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

uint64_t mb_master_timeout_error = 0;
float32_t mb_master_timeout_error_percent = 0;
uint64_t mb_master_crc_error = 0;
float32_t mb_master_crc_error_percent = 0;
uint64_t mb_master_request = 0;
uint64_t mb_master_response = 0;

TickType_t xTimeOutBefore, xTotalTimeOutSuspended;

uint16_t trigger_485_event_attribute_warning = 0;
uint16_t trigger_485_event_attribute_emerg = 0;

uint32_t trigger_485_event_attribute_xtd1 = 0;
uint32_t trigger_485_event_attribute_xtd2 = 0;
uint32_t trigger_485_event_attribute_xtd3 = 0;

uint64_t trigger_485_ZSK = 0; 
uint64_t trigger_485_ZSK_previous = 0; 
uint16_t trigger_485_ZSK_percent = 0;
uint16_t trigger_485_ZSK_percent_prev = 0;
uint64_t ZSK_trigger_array[ZSK_REG_485_QTY];
uint64_t ZSK_trigger_array_previous[ZSK_REG_485_QTY];

volatile int x_axis = 0;
volatile int y_axis = 0; 
volatile int z_axis = 0;

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
float32_t baud_rate_uart_1 = 115200;	//master Ex (XP2)
float32_t baud_rate_uart_2 = 115200; //slave Ex (XP6)
float32_t baud_rate_uart_3 = 115200; //slave (TBUS)
uint8_t bootloader_state = 0;
extern uint32_t boot_timer_counter;	
uint16_t trigger_event_attribute = 0;
uint32_t trigger_485_mode_2 = 0;


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

volatile uint32_t byte_size = 0;
volatile uint16_t crc_data = 0;
volatile uint16_t byte_bunch = 0;
volatile uint32_t byte_counter = 0;
volatile uint16_t crc_flash = 0;
volatile uint64_t data = 0;
uint8_t error_crc = 0;
volatile uint8_t worker_status = 0;
volatile uint8_t status = 0;
volatile uint8_t status1 = 0;
volatile uint8_t status2 = 0;
volatile uint8_t status3 = 0;

uint16_t size_moving_average_ZSK;
float32_t* zsk_average_array[ZSK_REG_485_QTY]; 
float32_t average_result = 0.0;

//volatile uint16_t reg_lost_packet = 0;
volatile uint16_t reg_lost_packet[REG_485_QTY];



/* USER CODE END Variables */
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
osThreadId myTask23Handle;
osThreadId myTask24Handle;
osThreadId myTask25Handle;
osThreadId myTask26Handle;

/* Private function prototypes -----------------------------------------------*/
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
void JumpToApplication(uint32_t ADDRESS);
/* USER CODE END FunctionPrototypes */

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
void TBUS_Modbus_Receive_Task(void const * argument);
void TBUS_Modbus_Transmit_Task(void const * argument);
void Update_Bootloader_Task(void const * argument);
void Erase_Bootloader_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	//Время усреднения выборки (4с.=64, 2с.=32, 1с.=16)
	if (filter_mode_icp == 0) QUEUE_LENGHT = 200;
	else QUEUE_LENGHT = 200;
	
	
	
	Q_A_rms_array_icp = pvPortMalloc( sizeof(float32_t)*QUEUE_LENGHT );
	Q_V_rms_array_icp = pvPortMalloc( sizeof(float32_t)*QUEUE_LENGHT );
	Q_D_rms_array_icp = pvPortMalloc( sizeof(float32_t)*QUEUE_LENGHT );
	Q_A_mean_array_4_20 = pvPortMalloc( sizeof(float32_t)*QUEUE_LENGHT_4_20 );
	Q_A_peak_array_icp = pvPortMalloc( sizeof(float32_t)*QUEUE_LENGHT );
	Q_V_peak_array_icp = pvPortMalloc( sizeof(float32_t)*QUEUE_LENGHT );
	Q_D_peak_array_icp = pvPortMalloc( sizeof(float32_t)*QUEUE_LENGHT );
	Q_A_2peak_array_icp = pvPortMalloc( sizeof(float32_t)*QUEUE_LENGHT );
	Q_V_2peak_array_icp = pvPortMalloc( sizeof(float32_t)*QUEUE_LENGHT );
	Q_D_2peak_array_icp = pvPortMalloc( sizeof(float32_t)*QUEUE_LENGHT );
	Q_peak_array_4_20 = pvPortMalloc( sizeof(float32_t)*QUEUE_LENGHT_4_20 );
	Q_2peak_array_4_20 = pvPortMalloc( sizeof(float32_t)*QUEUE_LENGHT_4_20 );
	
	
	//Выделяем память под двумерный массив для усреднения (ЗСК)
	for (int i=0; i < ZSK_REG_485_QTY; i++) 
	{
		zsk_average_array[i] = (float32_t*) pvPortMalloc( size_moving_average_ZSK * sizeof(float32_t) );		
	}
		
	acceleration_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));	
	velocity_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	displacement_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));	
	queue_4_20 = xQueueCreate(QUEUE_LENGHT_4_20, sizeof(float32_t));	
	queue_peak_4_20 = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	queue_2peak_4_20 = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));

	
	acceleration_peak_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	velocity_peak_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	displacement_peak_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	acceleration_2peak_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	velocity_2peak_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));
	displacement_2peak_queue_icp = xQueueCreate(QUEUE_LENGHT, sizeof(float32_t));	


	
	vSemaphoreCreateBinary(Semaphore1);
	vSemaphoreCreateBinary(Semaphore2);
	vSemaphoreCreateBinary(Semaphore_Acceleration);
	vSemaphoreCreateBinary(Semaphore_Velocity);
	vSemaphoreCreateBinary(Semaphore_Displacement);
	vSemaphoreCreateBinary(Q_Semaphore_Acceleration);
	vSemaphoreCreateBinary(Q_Semaphore_Velocity);
	vSemaphoreCreateBinary(Q_Semaphore_Displacement);
	vSemaphoreCreateBinary(Semaphore_Modbus_Rx);
	vSemaphoreCreateBinary(Semaphore_Modbus_Tx);
	vSemaphoreCreateBinary(Semaphore_Master_Modbus_Rx);
	vSemaphoreCreateBinary(Semaphore_Master_Modbus_Tx);
	vSemaphoreCreateBinary(Semaphore_Relay_1);
	vSemaphoreCreateBinary(Semaphore_Relay_2);
	vSemaphoreCreateBinary(Semaphore_HART_Receive);
	vSemaphoreCreateBinary(Semaphore_HART_Transmit);
	Mutex_Setting = xSemaphoreCreateMutex();       
	vSemaphoreCreateBinary(Semaphore_TBUS_Modbus_Rx);
	vSemaphoreCreateBinary(Semaphore_TBUS_Modbus_Tx);
	vSemaphoreCreateBinary(Semaphore_Bootloader_Update);
	vSemaphoreCreateBinary(Semaphore_Bootloader_Erase);
	Mutex_Average = xSemaphoreCreateMutex();       
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, Acceleration_Task, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, Velocity_Task, osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, Displacement_Task, osPriorityNormal, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, Q_Average_A, osPriorityNormal, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, Q_Average_V, osPriorityNormal, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* definition and creation of myTask07 */
  osThreadDef(myTask07, Q_Average_D, osPriorityNormal, 0, 128);
  myTask07Handle = osThreadCreate(osThread(myTask07), NULL);

  /* definition and creation of myTask08 */
  osThreadDef(myTask08, ADC_supply_voltage, osPriorityNormal, 0, 128);
  myTask08Handle = osThreadCreate(osThread(myTask08), NULL);

  /* definition and creation of myTask09 */
  osThreadDef(myTask09, Lights_Task, osPriorityNormal, 0, 128);
  myTask09Handle = osThreadCreate(osThread(myTask09), NULL);

  /* definition and creation of myTask10 */
  osThreadDef(myTask10, DAC_Task, osPriorityNormal, 0, 128);
  myTask10Handle = osThreadCreate(osThread(myTask10), NULL);

  /* definition and creation of myTask11 */
  osThreadDef(myTask11, Display_Task, osPriorityNormal, 0, 128);
  myTask11Handle = osThreadCreate(osThread(myTask11), NULL);

  /* definition and creation of myTask12 */
  osThreadDef(myTask12, Button_Task, osPriorityNormal, 0, 128);
  myTask12Handle = osThreadCreate(osThread(myTask12), NULL);

  /* definition and creation of myTask13 */
  osThreadDef(myTask13, Modbus_Receive_Task, osPriorityNormal, 0, 128);
  myTask13Handle = osThreadCreate(osThread(myTask13), NULL);

  /* definition and creation of myTask14 */
  osThreadDef(myTask14, Modbus_Transmit_Task, osPriorityNormal, 0, 128);
  myTask14Handle = osThreadCreate(osThread(myTask14), NULL);

  /* definition and creation of myTask15 */
  osThreadDef(myTask15, Master_Modbus_Receive, osPriorityNormal, 0, 128);
  myTask15Handle = osThreadCreate(osThread(myTask15), NULL);

  /* definition and creation of myTask16 */
  osThreadDef(myTask16, Master_Modbus_Transmit, osPriorityNormal, 0, 128);
  myTask16Handle = osThreadCreate(osThread(myTask16), NULL);

  /* definition and creation of myTask17 */
  osThreadDef(myTask17, Data_Storage_Task, osPriorityNormal, 0, 128);
  myTask17Handle = osThreadCreate(osThread(myTask17), NULL);

  /* definition and creation of myTask18 */
  osThreadDef(myTask18, TriggerLogic_Task, osPriorityNormal, 0, 128);
  myTask18Handle = osThreadCreate(osThread(myTask18), NULL);

  /* definition and creation of myTask19 */
  osThreadDef(myTask19, Relay_1_Task, osPriorityNormal, 0, 128);
  myTask19Handle = osThreadCreate(osThread(myTask19), NULL);

  /* definition and creation of myTask20 */
  osThreadDef(myTask20, Relay_2_Task, osPriorityNormal, 0, 128);
  myTask20Handle = osThreadCreate(osThread(myTask20), NULL);

  /* definition and creation of myTask21 */
  osThreadDef(myTask21, HART_Receive_Task, osPriorityNormal, 0, 128);
  myTask21Handle = osThreadCreate(osThread(myTask21), NULL);

  /* definition and creation of myTask22 */
  osThreadDef(myTask22, HART_Transmit_Task, osPriorityNormal, 0, 128);
  myTask22Handle = osThreadCreate(osThread(myTask22), NULL);

  /* definition and creation of myTask23 */
  osThreadDef(myTask23, TBUS_Modbus_Receive_Task, osPriorityNormal, 0, 128);
  myTask23Handle = osThreadCreate(osThread(myTask23), NULL);

  /* definition and creation of myTask24 */
  osThreadDef(myTask24, TBUS_Modbus_Transmit_Task, osPriorityNormal, 0, 128);
  myTask24Handle = osThreadCreate(osThread(myTask24), NULL);

  /* definition and creation of myTask25 */
  osThreadDef(myTask25, Update_Bootloader_Task, osPriorityNormal, 0, 128);
  myTask25Handle = osThreadCreate(osThread(myTask25), NULL);

  /* definition and creation of myTask26 */
  osThreadDef(myTask26, Erase_Bootloader_Task, osPriorityNormal, 0, 128);
  myTask26Handle = osThreadCreate(osThread(myTask26), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
	
	Task_manager_Init();
	
  /* Infinite loop */
  for(;;)
  {
		Task_manager_LoadCPU();		
		
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Acceleration_Task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Acceleration_Task */
void Acceleration_Task(void const * argument)
{
  /* USER CODE BEGIN Acceleration_Task */
	
	
	float32_t temp_rms_acceleration_icp = 0.0;
	float32_t temp_mean_acceleration_4_20 = 0.0;	
	float32_t temp_max_acceleration_4_20 = 0.0;
	float32_t temp_min_acceleration_4_20 = 0.0;	
	float32_t temp_max_acceleration_icp = 0.0;	
	float32_t temp_min_acceleration_icp = 0.0;	
	uint32_t index;	
	float32_t constant_voltage;
	
	
  /* Infinite loop */
  for(;;)
  {		

		xSemaphoreTake( Semaphore_Acceleration, portMAX_DELAY );	

		
		
		//Получаем данные
		if (adc_bunch == 1)
		{			
			for (uint16_t i=0, k=0; i < RAW_ADC_BUFFER_SIZE/2; i=i+2, k++)
			{			
				float_adc_value_ICP[k] = raw_adc_value[i];
				
				float_adc_value_4_20[k] = raw_adc_value[i+1];							
			}			

			bunch_count_1++;			
		}		
		else if (adc_bunch == 2)
		{			
			for (uint16_t i=RAW_ADC_BUFFER_SIZE/2, k=0; i < RAW_ADC_BUFFER_SIZE; i=i+2, k++) 
			{
				float_adc_value_ICP[k] = raw_adc_value[i];	

				float_adc_value_4_20[k] = raw_adc_value[i+1];							
			}
			
			bunch_count_2++;						
		}
	

		//Усредняем постоянку ICP
		arm_rms_f32( (float32_t*)&float_adc_value_ICP[0], ADC_BUFFER_SIZE, (float32_t*)&constant_voltage );
				
		
		//Фильтр НЧ (lpf 25600)
		arm_biquad_cascade_df1_f32(&filter_main_low_icp, (float32_t*) &float_adc_value_ICP[0], (float32_t*) &float_adc_value_ICP[0], ADC_BUFFER_SIZE);										
		//arm_biquad_cascade_df1_f32(&filter_main_low_4_20, (float32_t*) &float_adc_value_4_20[0], (float32_t*) &float_adc_value_4_20[0], ADC_BUFFER_SIZE);			
		
		//Дециматор 25600 -> 6400
		for (uint16_t i=0, j=0; i < ADC_BUFFER_SIZE; i=i+4, j++) 
		{
			float_adc_value_ICP_64_1[j] = float_adc_value_ICP[i];
		}		

		//Фильтр ВЧ (hpf)
		arm_biquad_cascade_df1_f32(&filter_main_high_icp, (float32_t*) &float_adc_value_ICP_64_1[0], (float32_t*) &float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL);		
		
		//Фильтр НЧ (lpf 6400)
		arm_biquad_cascade_df1_f32(&filter_main_low_icp_2, (float32_t*) &float_adc_value_ICP_64_1[0], (float32_t*) &float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL);										
		
		//СКЗ
		arm_rms_f32( (float32_t*)&float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL, (float32_t*)&temp_rms_acceleration_icp );
		arm_rms_f32( (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, (float32_t*)&temp_mean_acceleration_4_20 );
		
		//Max
		arm_max_f32( (float32_t*)&float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL, (float32_t*)&temp_max_acceleration_icp, &index );
		arm_max_f32( (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, (float32_t*)&temp_max_acceleration_4_20, &index );
				
		//Min
		arm_min_f32( (float32_t*)&float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL, (float32_t*)&temp_min_acceleration_icp, &index );
		arm_min_f32( (float32_t*)&float_adc_value_4_20[0], ADC_BUFFER_SIZE, (float32_t*)&temp_min_acceleration_4_20, &index );
		
		
		xQueueSend(acceleration_queue_icp, (void*)&temp_rms_acceleration_icp, 0);				
		xQueueSend(queue_4_20, (void*)&temp_mean_acceleration_4_20, 0);		
		
//		xQueueSend(acceleration_peak_queue_icp, (void*)&temp_max_acceleration_icp, 0);				
//		xQueueSend(acceleration_2peak_queue_icp, (void*)&temp_min_acceleration_icp, 0);
		max_acceleration_icp = temp_max_acceleration_icp * icp_coef_K + icp_coef_B;
		min_acceleration_icp = temp_min_acceleration_icp * icp_coef_K + icp_coef_B;

		xQueueSend(queue_peak_4_20, (void*)&temp_max_acceleration_4_20, 0);				
		xQueueSend(queue_2peak_4_20, (void*)&temp_min_acceleration_4_20, 0);
		
		
		

		//Детектор обрыва ICP (0 - нет обрыва, 1 - обрыв)
		if ( constant_voltage > 4000 ) break_sensor_icp = 1;
		else break_sensor_icp = 0;

		//Детектор обрыва 4-20 (0 - нет обрыва, 1 - обрыв)
		if ( (temp_mean_acceleration_4_20 * coef_ampl_420 + coef_offset_420) > break_level_4_20 ) break_sensor_420 = 0;
		else break_sensor_420 = 1;
		
		
		//Счетчик оборотов
		turnover_counter( &float_adc_value_4_20[0] );
		

		xSemaphoreGive( Semaphore_Velocity );
		xSemaphoreGive( Q_Semaphore_Acceleration );		
		
		
  }
  /* USER CODE END Acceleration_Task */
}

/* USER CODE BEGIN Header_Velocity_Task */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Velocity_Task */
void Velocity_Task(void const * argument)
{
  /* USER CODE BEGIN Velocity_Task */
	
	float32_t temp_rms_velocity_icp = 0.0;	
	float32_t temp_max_velocity_icp = 0.0;	
	float32_t temp_min_velocity_icp = 0.0;
	
			
	uint32_t index;
	
  
	/* Infinite loop */
  for(;;)
  {
    xSemaphoreTake( Semaphore_Velocity, portMAX_DELAY );
			
		
		//Интегратор
		Integrate_V( (float32_t*)&float_adc_value_ICP_64_1[0], (float32_t*)&float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL );		
		
		//Фильтр ВЧ (hpf)
		arm_biquad_cascade_df1_f32(&filter_instance_highpass_1_icp, (float32_t*) &float_adc_value_ICP_64_1[0], (float32_t*) &float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL);		

		
				
		//СКЗ
		arm_rms_f32( (float32_t*)&float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL, (float32_t*)&temp_rms_velocity_icp );
		
		//Max
		arm_max_f32( (float32_t*)&float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL, (float32_t*)&temp_max_velocity_icp, &index );				
				
		//Min
		arm_min_f32( (float32_t*)&float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL, (float32_t*)&temp_min_velocity_icp, &index );
				
		
		
		xQueueSend(velocity_queue_icp, (void*)&temp_rms_velocity_icp, 0);	

//		xQueueSend(velocity_peak_queue_icp, (void*)&temp_max_velocity_icp, 0);	
//		xQueueSend(velocity_2peak_queue_icp, (void*)&temp_min_velocity_icp, 0);	
		max_velocity_icp = (float32_t) (temp_max_velocity_icp * icp_coef_K + icp_coef_B);
		min_velocity_icp = (float32_t) (temp_min_velocity_icp * icp_coef_K + icp_coef_B);
		
		xSemaphoreGive( Semaphore_Displacement );
		xSemaphoreGive( Q_Semaphore_Velocity );		

		
  }
  /* USER CODE END Velocity_Task */
}

/* USER CODE BEGIN Header_Displacement_Task */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Displacement_Task */
void Displacement_Task(void const * argument)
{
  /* USER CODE BEGIN Displacement_Task */
	
	float32_t temp_rms_displacement_icp = 0.0;		
	float32_t temp_max_displacement_icp = 0.0;		
	float32_t temp_min_displacement_icp = 0.0;	
			
	uint32_t index;	
	
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake( Semaphore_Displacement, portMAX_DELAY );				
		
		
		//Интегратор			
		Integrate_D( (float32_t*)&float_adc_value_ICP_64_1[0], (float32_t*)&float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL );		
				
		//Фильтр ВЧ
		arm_biquad_cascade_df1_f32(&filter_instance_highpass_2_icp, (float32_t*) &float_adc_value_ICP_64_1[0], (float32_t*) &float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL);		
		
		
		//СКЗ
		arm_rms_f32( (float32_t*)&float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL, (float32_t*)&temp_rms_displacement_icp );								
		
		//Max
		arm_max_f32( (float32_t*)&float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL, (float32_t*)&temp_max_displacement_icp, &index );
				
		//Min
		arm_min_f32( (float32_t*)&float_adc_value_ICP_64_1[0], ADC_BUFFER_SIZE_SMALL, (float32_t*)&temp_min_displacement_icp, &index );
				
								
		xQueueSend(displacement_queue_icp, (void*)&temp_rms_displacement_icp, 0);
		
//		xQueueSend(displacement_peak_queue_icp, (void*)&temp_max_displacement_icp, 0);
//		xQueueSend(displacement_2peak_queue_icp, (void*)&temp_min_displacement_icp, 0);		
		max_displacement_icp = (float32_t) (temp_max_displacement_icp  * icp_coef_K + icp_coef_B);
		min_displacement_icp = (float32_t) (temp_min_displacement_icp * icp_coef_K + icp_coef_B);
		
		xSemaphoreGive( Q_Semaphore_Displacement );
		
  }
  /* USER CODE END Displacement_Task */
}

/* USER CODE BEGIN Header_Q_Average_A */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Q_Average_A */
void Q_Average_A(void const * argument)
{
  /* USER CODE BEGIN Q_Average_A */
	uint32_t index;
  /* Infinite loop */
  for(;;)
  {		
			xSemaphoreTake( Q_Semaphore_Acceleration, portMAX_DELAY );
				
			
			queue_count_A_icp = uxQueueMessagesWaiting(acceleration_queue_icp);	
				
			if (queue_count_A_icp == QUEUE_LENGHT)
			{						
					rms_acceleration_icp = 0.0;		
								
					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
					{
							xQueueReceive(acceleration_queue_icp, (void *) &Q_A_rms_array_icp[i], 0);										
					}
					
					arm_rms_f32( Q_A_rms_array_icp, QUEUE_LENGHT, (float32_t*)&rms_acceleration_icp);	
					
					icp_voltage  = rms_acceleration_icp * COEF_TRANSFORM_VOLT;
					
					
					//rms_acceleration_icp = (float32_t) COEF_TRANSFORM_icp_acceleration * icp_voltage;
					//rms_acceleration_icp = (float32_t) (icp_range_volt / icp_range_a) * icp_voltage;
					rms_acceleration_icp = rms_acceleration_icp * icp_coef_K + icp_coef_B;
					
					
//					max_acceleration_icp = 0.0;
//					min_acceleration_icp = 0.0;
//					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
//					{
//							xQueueReceive(acceleration_peak_queue_icp, (void *) &Q_A_peak_array_icp[i], 0);										
//							xQueueReceive(acceleration_2peak_queue_icp, (void *) &Q_A_2peak_array_icp[i], 0);										
//					}
//					arm_max_f32( (float32_t*)&Q_A_peak_array_icp[0], QUEUE_LENGHT, (float32_t*)&max_acceleration_icp, &index );
//					arm_min_f32( (float32_t*)&Q_A_2peak_array_icp[0], QUEUE_LENGHT, (float32_t*)&min_acceleration_icp, &index );
//					max_acceleration_icp = max_acceleration_icp * icp_coef_K + icp_coef_B;
//					min_acceleration_icp = min_acceleration_icp * icp_coef_K + icp_coef_B;
			}
				
				
				
			queue_count_A_4_20 = uxQueueMessagesWaiting(queue_4_20);		

			if (queue_count_A_4_20 == QUEUE_LENGHT_4_20)
			{						
					mean_4_20 = 0.0;		
								
					for (uint16_t i=0; i<QUEUE_LENGHT_4_20; i++)
					{
							xQueueReceive(queue_4_20, (void *) &Q_A_mean_array_4_20[i], 0);										
					}					
					arm_rms_f32( Q_A_mean_array_4_20, QUEUE_LENGHT_4_20, (float32_t*)&mean_4_20 );																
						
					//Усредненное значение тока
					mean_4_20 = (float32_t) (mean_4_20 * coef_ampl_420 + coef_offset_420);

					//Пересчет тока в пользовательский диапазон
					//calculated_value_4_20 =  (mean_4_20 - 4.0) * (16.0 / (up_user_range_4_20 - down_user_range_4_20));
					calculated_value_4_20 =  down_user_range_4_20 + (up_user_range_4_20 - down_user_range_4_20) * ((mean_4_20 - 4.0) / (20.0 - 4.0));
					
					
					max_4_20 = 0.0;
					min_4_20 = 0.0;					
					for (uint16_t i=0; i<QUEUE_LENGHT_4_20; i++)
					{
							xQueueReceive(queue_peak_4_20, (void *) &Q_peak_array_4_20[i], 0);										
							xQueueReceive(queue_2peak_4_20, (void *) &Q_2peak_array_4_20[i], 0);										
					}
					arm_max_f32( (float32_t*)&Q_peak_array_4_20[0], QUEUE_LENGHT_4_20, (float32_t*)&max_4_20, &index );
					arm_min_f32( (float32_t*)&Q_2peak_array_4_20[0], QUEUE_LENGHT_4_20, (float32_t*)&min_4_20, &index );
					max_4_20 = (float32_t) max_4_20 * coef_ampl_420 + coef_offset_420;
					min_4_20 = (float32_t) min_4_20 * coef_ampl_420 + coef_offset_420;						
			}

				
  }
  /* USER CODE END Q_Average_A */
}

/* USER CODE BEGIN Header_Q_Average_V */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Q_Average_V */
void Q_Average_V(void const * argument)
{
  /* USER CODE BEGIN Q_Average_V */
	uint32_t index;
  /* Infinite loop */
  for(;;)
  {
			xSemaphoreTake( Q_Semaphore_Velocity, portMAX_DELAY );
			
			queue_count_V_icp = uxQueueMessagesWaiting(velocity_queue_icp);		
			
			if (queue_count_V_icp == QUEUE_LENGHT)
			{						
					rms_velocity_icp = 0.0;		
								
					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
					{
							xQueueReceive(velocity_queue_icp, (void *) &Q_V_rms_array_icp[i], 0);										
					}
					
					arm_rms_f32( Q_V_rms_array_icp, QUEUE_LENGHT, (float32_t*)&rms_velocity_icp );
										
						
					rms_velocity_icp = (float32_t) (rms_velocity_icp * icp_coef_K + icp_coef_B);		

					//Вычисление разницы времени между проходами
					xTotalTimeSuspended = xTaskGetTickCount() - xTimeBefore;
					xTimeBefore = xTaskGetTickCount();	
										
			
			
			
//					max_velocity_icp = 0.0;
//					min_velocity_icp = 0.0;
//					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
//					{
//							xQueueReceive(velocity_peak_queue_icp, (void *) &Q_V_peak_array_icp[i], 0);										
//							xQueueReceive(velocity_2peak_queue_icp, (void *) &Q_V_2peak_array_icp[i], 0);										
//					}
//					arm_max_f32( (float32_t*)&Q_V_peak_array_icp[0], QUEUE_LENGHT, (float32_t*)&max_velocity_icp, &index );
//					arm_min_f32( (float32_t*)&Q_V_2peak_array_icp[0], QUEUE_LENGHT, (float32_t*)&min_velocity_icp, &index );
//					max_velocity_icp = (float32_t) (max_velocity_icp * icp_coef_K + icp_coef_B);
//					min_velocity_icp = (float32_t) (min_velocity_icp * icp_coef_K + icp_coef_B);
			}

  }
  /* USER CODE END Q_Average_V */
}

/* USER CODE BEGIN Header_Q_Average_D */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Q_Average_D */
void Q_Average_D(void const * argument)
{
  /* USER CODE BEGIN Q_Average_D */
	uint32_t index;
  /* Infinite loop */
  for(;;)
  {
			xSemaphoreTake( Q_Semaphore_Displacement, portMAX_DELAY );
				
			queue_count_D_icp = uxQueueMessagesWaiting(displacement_queue_icp);		
			
			if (queue_count_D_icp == QUEUE_LENGHT)
			{						
					rms_displacement_icp = 0.0;		
								
					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
					{
							xQueueReceive(displacement_queue_icp, (void *) &Q_D_rms_array_icp[i], 0);										
					}
					
					arm_rms_f32( Q_D_rms_array_icp, QUEUE_LENGHT, (float32_t*)&rms_displacement_icp );

					rms_displacement_icp = (float32_t) (rms_displacement_icp * icp_coef_K + icp_coef_B);					
			 


//					max_displacement_icp = 0.0;
//					min_displacement_icp = 0.0;
//					for (uint16_t i=0; i<QUEUE_LENGHT; i++)
//					{
//							xQueueReceive(displacement_peak_queue_icp, (void *) &Q_D_peak_array_icp[i], 0);										
//							xQueueReceive(displacement_2peak_queue_icp, (void *) &Q_D_2peak_array_icp[i], 0);										
//					}
//					arm_max_f32( (float32_t*)&Q_D_peak_array_icp[0], QUEUE_LENGHT, (float32_t*)&max_displacement_icp, &index );
//					arm_min_f32( (float32_t*)&Q_D_2peak_array_icp[0], QUEUE_LENGHT, (float32_t*)&min_displacement_icp, &index );
//					max_displacement_icp = (float32_t) (max_displacement_icp  * icp_coef_K + icp_coef_B);
//					min_displacement_icp = (float32_t) (min_displacement_icp * icp_coef_K + icp_coef_B);
			}
  }
  /* USER CODE END Q_Average_D */
}

/* USER CODE BEGIN Header_ADC_supply_voltage */
/**
* @brief Function implementing the myTask08 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADC_supply_voltage */
void ADC_supply_voltage(void const * argument)
{
  /* USER CODE BEGIN ADC_supply_voltage */
	uint16_t supply_voltage = 0;
  /* Infinite loop */
  for(;;)
  {
		HAL_ADCEx_InjectedStart(&hadc1);
		HAL_ADCEx_InjectedPollForConversion(&hadc1, 100);
		supply_voltage = HAL_ADCEx_InjectedGetValue(&hadc1, 1);
		HAL_ADCEx_InjectedStop(&hadc1);
	
		power_supply_voltage = (float32_t) supply_voltage * COEF_TRANSFORM_SUPPLY;
								
    osDelay(1000);
  }
  /* USER CODE END ADC_supply_voltage */
}

/* USER CODE BEGIN Header_Lights_Task */
/**
* @brief Function implementing the myTask09 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Lights_Task */
void Lights_Task(void const * argument)
{
  /* USER CODE BEGIN Lights_Task */
  /* Infinite loop */
  for(;;)
  {
		//Прогрев
		if (warming_flag == 1) 
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			osDelay(300);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
			osDelay(300);						
		}
		else
		{	

			//Если реле не сработали и нет обрыва(по любому из каналов) и канал включен, то зажигаем зеленый
		
			if (( break_sensor_icp == 1 && channel_ICP_ON == 1) || (break_sensor_420 == 1 && channel_4_20_ON == 1) || (break_sensor_485 == 1 && channel_485_ON == 1 )) 
			{
				//Горит красный
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			}
			else 
			{				
				//Горит зеленый
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);	
			}
			
						
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0)
			{								
				//Мигает Синий 
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
				
				osDelay(500);
				
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
				
				osDelay(500);
				
			}			
			
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1)
			{				
				//Мигает Красный 
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
				
				osDelay(200);
				
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
				
				osDelay(200);
			}
			
		
			osDelay(100);			
		}
	}
  /* USER CODE END Lights_Task */
}

/* USER CODE BEGIN Header_DAC_Task */
/**
* @brief Function implementing the myTask10 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DAC_Task */
void DAC_Task(void const * argument)
{
  /* USER CODE BEGIN DAC_Task */
	volatile uint32_t out_dac = 0;
	volatile float32_t a_to_v = 0.0;
	float32_t variable_485 = 0.0;
	float32_t range_for_out = 0.0;
	volatile uint16_t reg_number_485 = 0;
	uint8_t f = 0;
  /* Infinite loop */
  for(;;)
  {
		//Диапазон
		range_for_out = convert_hex_to_float(&settings[0], 94);
		
		//Номер регистра канала 485 для выхода 4-20
		reg_number_485 = settings[97];
		
		//Источник сигнала "калибровочный регистр"
		if (settings[89] == 0)
		{
			variable_for_out_4_20 = convert_hex_to_float(&settings[0], 62);					
			out_required_current = variable_for_out_4_20;
		}		
		
		//Источник сигнала ICP
		if (settings[89] == 1)
		{						
			out_required_current = rms_velocity_icp * (16.0 / range_for_out) + 4;		
		}
		
		//Источник сигнала 4-20
		if (settings[89] == 2)
		{
			out_required_current = mean_4_20;
		}
		
		//Источник сигнала 485
		if (settings[89] == 3)
		{
			//variable_485 = master_array[reg_number_485].master_value; 

			out_required_current = master_array[reg_number_485].master_value * (16.0 / range_for_out) + 4;						
		}		
		
		//Меандр
		if (settings[89] == 4)
		{
			//variable_485 = master_array[reg_number_485].master_value; 
			
			out_required_current = (16.0 / range_for_out) + 10;			
			out_dac = (out_required_current * (4095 / 20)) * out_4_20_coef_K  + out_4_20_coef_B;
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t) out_dac);
			osDelay(1);
			
			out_required_current = (16.0 / range_for_out) + 4;
			out_dac = (out_required_current * (4095 / 20)) * out_4_20_coef_K  + out_4_20_coef_B;	
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t) out_dac);
			osDelay(19);

		}	
		else
		{
			//a_to_v = (float32_t) out_required_current * (3.3 / 20.00); 	
			//a_to_v = (3.3 * out_required_current / 4095.0);// * out_4_20_coef_K  + out_4_20_coef_B;
			
			
			out_dac = (out_required_current * (4095 / 20)) * out_4_20_coef_K  + out_4_20_coef_B;	
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t) out_dac);
			
			osDelay(100);
		}

    
  }
  /* USER CODE END DAC_Task */
}

/* USER CODE BEGIN Header_Display_Task */
/**
* @brief Function implementing the myTask11 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Display_Task */
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
	osDelay(settings[109]/2); //Заливка половина времени прогрева
	

	init_menu(1);
	
  /* Infinite loop */
  for(;;)
  {		
		
			if (warming_flag == 1) 
			{				
				ssd1306_Fill(0);				
				
				logo();
				
				ssd1306_UpdateScreen();
			}
			else if (bootloader_state == 1 && (worker_status == 1 || worker_status == 2 || worker_status == 3 || worker_status == 4) )
			{					
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("Обнов",font_8x15_RU,1);					
					ssd1306_WriteString("-",font_8x14,1);					
					ssd1306_SetCursor(0,15);	
					ssd1306_WriteString("ление",font_8x15_RU,1);					
					ssd1306_SetCursor(0,30);	
					ssd1306_WriteString("ПО",font_8x15_RU,1);					
					ssd1306_WriteString("...",font_8x14,1);
					ssd1306_UpdateScreen();
			}						
			else if (bootloader_state == 1 && worker_status == 5)
			{					
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,15);
					ssd1306_WriteString("УСПЕШНО",font_8x15_RU,1);										
						
					ssd1306_UpdateScreen();	
			}			
			else 				
			{							
					//Навигация по горизонтальному меню							
					if (menu_index_pointer == 1) //ICP
					{
						if (menu_edit_settings_mode == 0) horizont_menu_lenght = 9; 
						else horizont_menu_lenght = 6;
					}
					
					if (menu_index_pointer == 2) //4-20
					{						
						if (menu_edit_settings_mode == 0) horizont_menu_lenght = 1; 
						else horizont_menu_lenght = 8;
					}
					
					if (menu_index_pointer == 3) //485
					{
						if (menu_edit_settings_mode == 0) horizont_menu_lenght = REG_485_QTY;
						else horizont_menu_lenght = 2;	
					}
					
					if (menu_index_pointer == 4) //Реле
					{						
						if (menu_edit_settings_mode == 0) horizont_menu_lenght = 2;
						else horizont_menu_lenght = 4;	
					}
					
					if (menu_index_pointer == 5) horizont_menu_lenght = 4; //Настройки
					if (menu_index_pointer == 6) horizont_menu_lenght = 3; //Информация
					if (menu_index_pointer == 7) horizont_menu_lenght = 3; //Конфигурация
					
				
					if (button_left_pressed_in == 1 && menu_horizontal > 0 && menu_edit_mode == 0) 
					{				
						menu_horizontal--;
						button_left_pressed_in = 0;
						button_center_pressed_in_short = 0;				
						digit_rank = 0;						
					}						
					
					if (button_right_pressed_in == 1 && menu_horizontal < horizont_menu_lenght && menu_edit_mode == 0) 					
					{				
						menu_horizontal++;
						button_right_pressed_in = 0;
						button_center_pressed_in_short = 0;
						digit_rank = 0;						
					}	
					
					//Навигация по вертикальному меню
					if (button_up_pressed_in == 1 && menu_index_pointer > 0 && button_center_pressed_in_short == 0 && menu_edit_mode == 0) 										
					{				
						
						if (menu_index_pointer == 3 && menu_horizontal != 0) //меню 485
						{
								if (menu_vertical > 0) menu_vertical--;
								button_up_pressed_in = 0;
								digit_rank = 0;
						}
						else
						{						
								if (menu_index > 0) menu_index--;					
								menu_index_pointer = menu_index_array[menu_index];
								
								button_up_pressed_in = 0;						
								menu_horizontal = 0;					
								digit_rank = 0;						
						}
					}						
						
					if (button_down_pressed_in == 1 && button_center_pressed_in_short == 0 && menu_edit_mode == 0) 					
					{						

						if (menu_index_pointer == 3 && menu_horizontal != 0) //меню 485
						{
									if (menu_vertical < REG_485_QTY && menu_vertical < 12) menu_vertical++;
									button_down_pressed_in = 0;
									digit_rank = 0;
						}						
						else
						{
								if (menu_index < number_of_items_in_the_menu-1) menu_index++;
								menu_index_pointer = menu_index_array[menu_index];		
								
								button_down_pressed_in = 0;																		
								menu_horizontal = 0;						
								digit_rank = 0;						
						}
					}	
					
					//При коротком нажатии включаем/выключаем режим редактирования, но не в гл.меню
					if (button_center_pressed_in_short == 1 && menu_horizontal != 0) 
					{
						menu_edit_mode = !menu_edit_mode;	
						button_center_pressed_in_short = 0;						
					}					
					//При длинном нажатии в гл.меню включаем/выключаем настроечный режим  
					else if (button_center_pressed_in_long == 1 && menu_horizontal == 0) 
					{
						menu_edit_settings_mode = !menu_edit_settings_mode;	
						button_center_pressed_in_long = 0;				
						quit_relay_button = 1; //Включаем таймер чтоб не срабатывало квитирование 						
					}
					
					//Переход между разрядами числа в режиме редактирования
					if (button_left_pressed_in == 1 && menu_edit_mode == 1) 
					{				
						if (digit_rank > 0) digit_rank--;
						else digit_rank = 0;
						
						button_left_pressed_in = 0;	
					}						
					
					if (button_right_pressed_in == 1 && menu_edit_mode == 1) 					
					{				
						if (digit_rank < 1) digit_rank++;
						else digit_rank = 1;
						
						button_right_pressed_in = 0;						
					}	
					
					//Сохранение настроек на флеш
					if (button_center_pressed_in_long == 1 && menu_horizontal != 0)
					{
						save_settings();
						button_center_pressed_in_long = 0;
						button_center_pressed_in_short = 0;
						menu_edit_settings_mode = 0;
						quit_relay_button = 1; //Включаем таймер чтобы не срабатывало квитирование						
					}
					
					
					
//////////ICP menu					
					if (channel_ICP_ON == 1)
					{	
							if (menu_index_pointer == 1 && menu_horizontal == 0)
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);		
								
								ssd1306_WriteString("ICP",font_8x14,1);										
														
								if (break_sensor_icp == 1) //Если обрыв
								{									
										if (temp_stat_1 == 0) 
										{
											ssd1306_SetCursor(0,15);											
											ssd1306_WriteString("ОБРЫВ",font_8x15_RU,1);
											ssd1306_SetCursor(0,30);	
											ssd1306_WriteString("ДАТЧИКА",font_8x15_RU,1);
										}
										else ssd1306_WriteString(" ",font_8x14,1);
										
										if (menu_edit_settings_mode == 1) triangle_right(55,0);
										triangle_right(60,0);																					
										triangle_down(58,43);
								}
								else
								{

									if (menu_edit_settings_mode == 0) //Режим просмотра вибропараметров ">"
									{
										ssd1306_Fill(0);
										ssd1306_SetCursor(0,0);												
										ssd1306_WriteString("ICP",font_8x14,1);										
										ssd1306_SetCursor(28,0);																														

										triangle_right(60,0);																					
										triangle_down(58,43);
										
										ssd1306_SetCursor(0,15);																									
									}
									else if (menu_edit_settings_mode == 1) //Режим настройки канала ">>"
									{
										ssd1306_Fill(0);
										ssd1306_SetCursor(0,0);												
										ssd1306_WriteString("ICP",font_8x14,1);										
										ssd1306_SetCursor(28,0);																														

										triangle_right(55,0);
										triangle_right(60,0);																					
										triangle_down(58,43);
										
										ssd1306_SetCursor(0,15);
									}										
									

									if (icp_menu_points_for_showing == 1)	
									{
										strncpy(msg,"СКЗ виброускорения", 18);
										string_scroll(msg, 18);								
										ssd1306_SetCursor(0,32);				
										snprintf(buffer, sizeof buffer, "%.01f", rms_acceleration_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}
									
									if (icp_menu_points_for_showing == 2)	
									{
										strncpy(msg,"СКЗ виброскорости", 17);
										string_scroll(msg, 17);									
										ssd1306_SetCursor(0,32);				
										snprintf(buffer, sizeof buffer, "%.01f", rms_velocity_icp);
										ssd1306_WriteString(buffer,font_8x14,1);		
									}										

									if (icp_menu_points_for_showing == 3)	
									{
										strncpy(msg,"СКЗ виброперемещения", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,32);				
										snprintf(buffer, sizeof buffer, "%.01f", rms_displacement_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}			

									if (icp_menu_points_for_showing == 4)	
									{
										strncpy(msg,"Амплитуда виброускорения", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,32);				
										snprintf(buffer, sizeof buffer, "%.01f", max_acceleration_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}	
									
									if (icp_menu_points_for_showing == 5)	
									{
										strncpy(msg,"Амплитуда виброскорости", 23);
										string_scroll(msg, 23);								
										ssd1306_SetCursor(0,32);				
										snprintf(buffer, sizeof buffer, "%.01f", max_velocity_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}										

									if (icp_menu_points_for_showing == 6)	
									{
										strncpy(msg,"Амплитуда виброперемещения", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,32);				
										snprintf(buffer, sizeof buffer, "%.01f", max_displacement_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}		
									
									
									if (icp_menu_points_for_showing == 7)	
									{
										strncpy(msg,"Размах виброускорения", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,32);				
										snprintf(buffer, sizeof buffer, "%.01f", max_acceleration_icp - min_acceleration_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}	
									
									if (icp_menu_points_for_showing == 8)	
									{
										strncpy(msg,"Размах виброскорости", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,30);				
										snprintf(buffer, sizeof buffer, "%.01f", max_velocity_icp - min_velocity_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}										

									if (icp_menu_points_for_showing == 9)	
									{
										strncpy(msg,"Размах виброперемещения", 20);
										string_scroll(msg, 20);								
										ssd1306_SetCursor(0,32);				
										snprintf(buffer, sizeof buffer, "%.01f", max_displacement_icp - min_displacement_icp);
										ssd1306_WriteString(buffer,font_8x14,1);											
									}
								}								
																
								//ssd1306_UpdateScreen();				
								
								menu_edit_mode = 0 ; //Запрещаем редактирование
								
								disable_up_down_button = 0;
							}

							
							if (menu_index_pointer == 1 && menu_horizontal == 1 && menu_edit_settings_mode == 0)
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);										
								ssd1306_SetCursor(28,0);						
								triangle_left(55,0);						
								triangle_right(60,0);						
								
								ssd1306_SetCursor(0,15);											
								strncpy(msg,"СКЗ виброускорения", 18);
								string_scroll(msg, 18);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", rms_acceleration_icp);
								ssd1306_WriteString(buffer,font_8x14,1);							
								
								//ssd1306_UpdateScreen();			

								menu_edit_mode = 0 ; //Запрещаем редактирование				
								
								disable_up_down_button = 1; //Выключаем кнопки вверх вниз

							}						

							

							if (menu_index_pointer == 1 && menu_horizontal == 2 && menu_edit_settings_mode == 0)
							{								
									
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);										
								ssd1306_SetCursor(28,0);																		
								triangle_left(55,0);						
								triangle_right(60,0);						
								
								ssd1306_SetCursor(0,15);																																		
								strncpy(msg,"СКЗ виброскорости", 17);
								string_scroll(msg, 17);
									
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", rms_velocity_icp);
								ssd1306_WriteString(buffer,font_8x14,1);		
								
								//ssd1306_UpdateScreen();			

								menu_edit_mode = 0 ; //Запрещаем редактирование		
								
								disable_up_down_button = 1; //Выключаем кнопки вверх вниз

							}			

					
							if (menu_index_pointer == 1 && menu_horizontal == 3 && menu_edit_settings_mode == 0)
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);										
								ssd1306_SetCursor(28,0);						
								triangle_left(55,0);						
								triangle_right(60,0);					
								
								ssd1306_SetCursor(0,15);											
								strncpy(msg,"СКЗ виброперемещения", 20);
								string_scroll(msg, 20);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", rms_displacement_icp);
								ssd1306_WriteString(buffer,font_8x14,1);							
								
								//ssd1306_UpdateScreen();			

								menu_edit_mode = 0 ; //Запрещаем редактирование

								disable_up_down_button = 1; //Выключаем кнопки вверх вниз								

							}
							
							if (menu_index_pointer == 1 && menu_horizontal == 4 && menu_edit_settings_mode == 0) //Амплитуда ускорения
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(55,0);						
								triangle_right(60,0);				
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Амплитуда виброускорения", 24);						
								string_scroll(msg, 24);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", max_acceleration_icp);
								ssd1306_WriteString(buffer,font_8x14,1); 		
														
								//ssd1306_UpdateScreen();

								menu_edit_mode = 0 ; //Запрещаем редактирование		

								disable_up_down_button = 1; //Выключаем кнопки вверх вниз
							}							
							
							if (menu_index_pointer == 1 && menu_horizontal == 5 && menu_edit_settings_mode == 0) //Амплитуда скорости
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(55,0);						
								triangle_right(60,0);					
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Амплитуда виброскорости", 23);						
								string_scroll(msg, 23);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", max_velocity_icp);
								ssd1306_WriteString(buffer,font_8x14,1); 		
														
								//ssd1306_UpdateScreen();

								menu_edit_mode = 0 ; //Запрещаем редактирование		

								disable_up_down_button = 1; //Выключаем кнопки вверх вниз
							}

							if (menu_index_pointer == 1 && menu_horizontal == 6 && menu_edit_settings_mode == 0) //Амплитуда перемещения
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(55,0);						
								triangle_right(60,0);			
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Амплитуда виброперемещения", 26);						
								string_scroll(msg, 26);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", max_displacement_icp);
								ssd1306_WriteString(buffer,font_8x14,1); 		
														
								//ssd1306_UpdateScreen();

								menu_edit_mode = 0 ; //Запрещаем редактирование		

								disable_up_down_button = 1; //Выключаем кнопки вверх вниз
							}

							if (menu_index_pointer == 1 && menu_horizontal == 7 && menu_edit_settings_mode == 0) //Размах ускорения
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(55,0);						
								triangle_right(60,0);				
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Размах виброускорения", 21);						
								string_scroll(msg, 21);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", max_acceleration_icp - min_acceleration_icp);
								ssd1306_WriteString(buffer,font_8x14,1); 		
														
								//ssd1306_UpdateScreen();

								menu_edit_mode = 0 ; //Запрещаем редактирование	

								disable_up_down_button = 1; //Выключаем кнопки вверх вниз
							}						

							
							if (menu_index_pointer == 1 && menu_horizontal == 8 && menu_edit_settings_mode == 0) //Размах скорости
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(55,0);						
								triangle_right(60,0);				
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Размах виброскорости", 20);						
								string_scroll(msg, 20);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", max_velocity_icp - min_velocity_icp);
								ssd1306_WriteString(buffer,font_8x14,1); 		
														
								//ssd1306_UpdateScreen();

								menu_edit_mode = 0 ; //Запрещаем редактирование

								disable_up_down_button = 1; //Выключаем кнопки вверх вниз								
							}				

							
							if (menu_index_pointer == 1 && menu_horizontal == 9 && menu_edit_settings_mode == 0) //Размах перемещения
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(55,0);						
																							
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Размах виброперемещения", 23);						
								string_scroll(msg, 23);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.03f", max_displacement_icp - min_displacement_icp);
								ssd1306_WriteString(buffer,font_8x14,1); 		
														
								//ssd1306_UpdateScreen();	

								menu_edit_mode = 0 ; //Запрещаем редактирование

								disable_up_down_button = 1; //Выключаем кнопки вверх вниз								
							}							




							//Режим настройки ICP
							
							if (menu_index_pointer == 1 && menu_horizontal == 1 && menu_edit_settings_mode == 1) //Номер параметра для показа на гл. экране
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);												
								triangle_left(50,0);
								triangle_right(55,0);							
								triangle_right(60,0);		
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Параметр на главном меню", 24);						
								string_scroll(msg, 24);
								
								ssd1306_SetCursor(0,32);			
								
								if (menu_edit_mode == 1) //Режим редактирования
								{											
											edit_mode_int(&icp_menu_points_for_showing);			
											disable_up_down_button = 0;
								}
								else //Нормальный режим
								{
									snprintf(buffer, sizeof buffer, "%d", icp_menu_points_for_showing);
									ssd1306_WriteString(buffer,font_8x14,1); 
									
									disable_up_down_button = 1; //Выключаем кнопки вверх вниз
								}												
								
								//ssd1306_UpdateScreen();			
								
							}
							
							
							if (menu_index_pointer == 1 && menu_horizontal == 2 && menu_edit_settings_mode == 1) //Предупр. уставка
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);												
								triangle_left(50,0);
								triangle_right(55,0);							
								triangle_right(60,0);		
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Предупредительная уставка", 25);						
								string_scroll(msg, 25);
								
								ssd1306_SetCursor(0,32);			
								
								if (menu_edit_mode == 1) //Режим редактирования
								{
											
											edit_mode(&hi_warning_icp);
											disable_up_down_button = 0;
										
								}
								else //Нормальный режим
								{
									snprintf(buffer, sizeof buffer, "%.01f", hi_warning_icp);
									ssd1306_WriteString(buffer,font_8x14,1); 
									
									disable_up_down_button = 1; //Выключаем кнопки вверх вниз
								}												
								
								//ssd1306_UpdateScreen();				
							}					
							
							
							
							if (menu_index_pointer == 1 && menu_horizontal == 3 && menu_edit_settings_mode == 1) //Авар. уставка
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(50,0);
								triangle_right(55,0);							
								triangle_right(60,0);					
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Аварийная уставка", 17);						
								string_scroll(msg, 17);						
								ssd1306_SetCursor(0,32);

								if (menu_edit_mode == 1) //Режим редактирования
								{
									edit_mode(&hi_emerg_icp);
									disable_up_down_button = 0;
								}
								else //Нормальный режим
								{
									snprintf(buffer, sizeof buffer, "%.01f", hi_emerg_icp);
									ssd1306_WriteString(buffer,font_8x14,1); 
									
									disable_up_down_button = 1; //Выключаем кнопки вверх вниз
								}					
														
								//ssd1306_UpdateScreen();				
							}	
							
							
							
							if (menu_index_pointer == 1 && menu_horizontal == 4 && menu_edit_settings_mode == 1) //Режим цифрового фильтра
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(50,0);
								triangle_right(55,0);							
								triangle_right(60,0);	
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Режим фильтра", 13);						
								string_scroll(msg, 13);
								
								ssd1306_SetCursor(0,32);										
								
//								if (menu_edit_mode == 1) //Режим редактирования
//								{
//									edit_mode_int8(&filter_mode_icp);
//									disable_up_down_button = 0;
//								}
//								else //Нормальный режим
								
								{
									snprintf(buffer, sizeof buffer, "%d", filter_mode_icp);
									ssd1306_WriteString(buffer,font_8x14,1); 
									
									disable_up_down_button = 1; //Выключаем кнопки вверх вниз
								}					
														
								//ssd1306_UpdateScreen();				
							}							

							if (menu_index_pointer == 1 && menu_horizontal == 5 && menu_edit_settings_mode == 1) //Коэф. усиления
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(50,0);
								triangle_right(55,0);							
								triangle_right(60,0);	
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Коэффициент усиления", 20);						
								string_scroll(msg, 20);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.05f", icp_coef_K);										
								ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
														
								//ssd1306_UpdateScreen();				
								
								menu_edit_mode = 0 ; //Запрещаем редактирование					

								disable_up_down_button = 1; //Выключаем кнопки вверх вниз								
							}	

							
							if (menu_index_pointer == 1 && menu_horizontal == 6 && menu_edit_settings_mode == 1) //Коэф. смещения
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("ICP",font_8x14,1);	
								triangle_left(50,0);
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Коэффициент смещения", 20);						
								string_scroll(msg, 20);
								
								ssd1306_SetCursor(0,32);				
								snprintf(buffer, sizeof buffer, "%.05f", icp_coef_B);										
								ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
														
								//ssd1306_UpdateScreen();

								menu_edit_mode = 0 ; //Запрещаем редактирование			

								disable_up_down_button = 1; //Выключаем кнопки вверх вниз								
							}

					}
					
//////////4-20 menu		
					if (channel_4_20_ON == 1)
					{					
					
							if (menu_index_pointer == 2 && menu_horizontal == 0) 
							{
								
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);										
														
								if (break_sensor_420 == 1 & channel_4_20_ON == 1) //Символ обрыва
								{							
										if (temp_stat_1 == 0) 
										{
											ssd1306_SetCursor(0,15);											
											ssd1306_WriteString("ОБРЫВ",font_8x15_RU,1);
											ssd1306_SetCursor(0,30);	
											ssd1306_WriteString("ДАТЧИКА",font_8x15_RU,1);
										}
										else ssd1306_WriteString(" ",font_8x14,1);
										
										if (menu_edit_settings_mode == 1) triangle_right(55,0);
										triangle_right(60,0);											
										if (channel_ICP_ON == 1) triangle_up(58,38);											
										triangle_down(58,43);
								}
								else
								{
								
									if (menu_edit_settings_mode == 0)
									{										
										triangle_right(60,0);											
										if (channel_ICP_ON == 1) triangle_up(58,38);											
										triangle_down(58,43);
									}
									else		
									{
										triangle_right(55,0);
										triangle_right(60,0);											
										if (channel_ICP_ON == 1) triangle_up(58,38);											
										triangle_down(58,43);
									}	
									
									ssd1306_SetCursor(0,15);																									
									
																		
									strncpy(msg,"Расчетное значение", 18);						
									string_scroll(msg, 18);
									
									ssd1306_SetCursor(0,32);				
									
									snprintf(buffer, sizeof buffer, "%.03f", calculated_value_4_20);
									ssd1306_WriteString(buffer,font_8x14,1);							
									
								}
								//ssd1306_UpdateScreen();			

								disable_up_down_button = 0;								
							}			

							if (menu_index_pointer == 2 && menu_horizontal == 1 && menu_edit_settings_mode == 0)							
							{
								ssd1306_Fill(0);

								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(55,0);				
								
								ssd1306_SetCursor(0,15);	
								ssd1306_WriteString("Ток",font_8x15_RU,1);		

								ssd1306_SetCursor(0,32);
								snprintf(buffer, sizeof buffer, "%.02f", mean_4_20);
								ssd1306_WriteString(buffer,font_8x14,1);
								//ssd1306_UpdateScreen();				
								menu_edit_mode = 0 ; //Запрещаем редактирование			

								disable_up_down_button = 1;								
							}

						
							
							//Режим настройки канала 4-20
							if (menu_index_pointer == 2 && menu_horizontal == 1 && menu_edit_settings_mode == 1) //Нижний диапазон
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(50,0);						
								triangle_right(55,0);							
								triangle_right(60,0);					
								
								ssd1306_SetCursor(0,15);	
								strncpy(msg,"Нижний предел диапазона для пересчета", 37);						
								string_scroll(msg, 37);
								
								ssd1306_SetCursor(0,32);				
								
								
								if (menu_edit_mode == 1) //Режим редактирования
								{								
									edit_mode(&down_user_range_4_20);
									disable_up_down_button = 0;
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.01f", down_user_range_4_20);
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
									disable_up_down_button = 1;
								}
														
								ssd1306_UpdateScreen();				
							}	
							
							if (menu_index_pointer == 2 && menu_horizontal == 2 && menu_edit_settings_mode == 1) //Верхний диапазон
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(50,0);						
								triangle_right(55,0);							
								triangle_right(60,0);			
								
								ssd1306_SetCursor(0,15);	
								strncpy(msg,"Верхний предел диапазона для пересчета", 38);						
								string_scroll(msg, 38);
								
								ssd1306_SetCursor(0,32);				
								
								
								if (menu_edit_mode == 1) //Режим редактирования
								{								
									edit_mode(&up_user_range_4_20);
									disable_up_down_button = 0;
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.01f", up_user_range_4_20);
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
									disable_up_down_button = 1;
								}
														
								ssd1306_UpdateScreen();				
							}	
							
							
							
							if (menu_index_pointer == 2 && menu_horizontal == 3 && menu_edit_settings_mode == 1) //Уставка нижняя предупр.
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(50,0);						
								triangle_right(55,0);							
								triangle_right(60,0);		
								
								ssd1306_SetCursor(0,15);	
								strncpy(msg,"Уставка нижняя предупредительная", 32);						
								string_scroll(msg, 32);
								
								ssd1306_SetCursor(0,32);				
								
								
								if (menu_edit_mode == 1) //Режим редактирования
								{								
									edit_mode(&lo_warning_420);
									disable_up_down_button = 0;
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.01f", lo_warning_420);
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
									disable_up_down_button = 1;
								}
														
								//ssd1306_UpdateScreen();				
							}		
							
							
							if (menu_index_pointer == 2 && menu_horizontal == 4 && menu_edit_settings_mode == 1) //Уставка нижняя авар.
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(50,0);						
								triangle_right(55,0);							
								triangle_right(60,0);									
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Уставка нижняя аварийная", 24);						
								string_scroll(msg, 24);						
								
								ssd1306_SetCursor(0,32);				
												
								
								if (menu_edit_mode == 1) //Режим редактирования
								{
									edit_mode(&lo_emerg_420);
									disable_up_down_button = 0;
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.01f", lo_emerg_420);
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
									disable_up_down_button = 1;
								}
														
								//ssd1306_UpdateScreen();				
							}					

							if (menu_index_pointer == 2 && menu_horizontal == 5 && menu_edit_settings_mode == 1) //Уставка врехняя предупр.
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(50,0);						
								triangle_right(55,0);							
								triangle_right(60,0);		
								
								ssd1306_SetCursor(0,15);	
								strncpy(msg,"Уставка верхняя предупредительная", 33);						
								string_scroll(msg, 33);
								
								ssd1306_SetCursor(0,32);				
														
								if (menu_edit_mode == 1) //Режим редактирования
								{
									edit_mode(&hi_warning_420);
									disable_up_down_button = 0;
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.01f", hi_warning_420);			
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
									disable_up_down_button = 1;
								}
														
								//ssd1306_UpdateScreen();				
							}

							if (menu_index_pointer == 2 && menu_horizontal == 6 && menu_edit_settings_mode == 1) //Уставка врехняя авар.
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(50,0);						
								triangle_right(55,0);							
								triangle_right(60,0);		
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Уставка верхняя аварийная", 25);						
								string_scroll(msg, 25);
								
								ssd1306_SetCursor(0,32);				
								
								if (menu_edit_mode == 1) //Режим редактирования
								{
									edit_mode(&hi_emerg_420);
									disable_up_down_button = 0;
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.01f", hi_emerg_420);			
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
									disable_up_down_button = 1;
								}
														
								//ssd1306_UpdateScreen();				
							}					
							
							
							if (menu_index_pointer == 2 && menu_horizontal == 7 && menu_edit_settings_mode == 1)							
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(50,0);						
								triangle_right(55,0);							
								triangle_right(60,0);				
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Коэффициент усиления", 20);						
								string_scroll(msg, 20);		
								
								ssd1306_SetCursor(0,30);
								snprintf(buffer, sizeof buffer, "%.05f", coef_ampl_420);
								ssd1306_WriteString(buffer,font_8x14,1);
								//ssd1306_UpdateScreen();								
								
								menu_edit_mode = 0 ; //Запрещаем редактирование
								
								disable_up_down_button = 1;
							}

							if (menu_index_pointer == 2 && menu_horizontal == 8 && menu_edit_settings_mode == 1)							
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("4-20",font_8x14,1);	
								triangle_left(50,0);						
																
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Коэффициент смещения", 20);						
								string_scroll(msg, 20);		
								
								ssd1306_SetCursor(0,30);
								snprintf(buffer, sizeof buffer, "%.05f", coef_offset_420);
								ssd1306_WriteString(buffer,font_8x14,1);
								//ssd1306_UpdateScreen();				

								menu_edit_mode = 0 ; //Запрещаем редактирование		

								disable_up_down_button = 1;								
							}								
						
					}
					
//////////485 menu		
					if ( (channel_485_ON == 1) || (channel_485_ON == 2) )
					{
						
							

							if (menu_index_pointer == 3 && menu_horizontal == 0) //Значение регистра
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("485",font_8x14,1);		
								
								if (channel_485_ON == 2) // ЗСК
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									ssd1306_WriteString("ТИК",font_8x15_RU,1);		
									ssd1306_SetCursor(28,0);												
									ssd1306_WriteString("ЗСК",font_8x15_RU,1);		
								}
								
								if (menu_edit_settings_mode == 0)
								{									
									triangle_right(60,0);											
									if (channel_ICP_ON == 1 || channel_4_20_ON == 1) triangle_up(58,38);											
									triangle_down(58,43);
								}
								else		
								{
									triangle_right(55,0);
									triangle_right(60,0);											
									if (channel_ICP_ON == 1 || channel_4_20_ON == 1) triangle_up(58,38);											
									triangle_down(58,43);
								}								
														
								if (break_sensor_485 == 1) //Символ обрыва
								{							
										if (temp_stat_1 == 0) 
										{
											ssd1306_SetCursor(0,15);											
											ssd1306_WriteString("НЕТ",font_8x15_RU,1);
											ssd1306_SetCursor(0,30);	
											ssd1306_WriteString("СВЯЗИ",font_8x15_RU,1);
											
										}
										else ssd1306_WriteString(" ",font_8x14,1);
								}
								else
								{	
										ssd1306_SetCursor(0,15);
										
										if (channel_485_ON == 2) // ЗСК	
										{										
											
											strncpy(msg,"Состояние", 9);						
											string_scroll(msg, 9);	
											
											ssd1306_SetCursor(0,32);				
											snprintf(buffer, sizeof buffer, "%d %%", trigger_485_ZSK_percent);
											ssd1306_WriteString(buffer,font_8x14,1);
											
										}
										else
										{
											strncpy(msg,"Значение регистра ", 18);
											string_scroll_with_number(msg, 18, menu_485_points_for_showing);					

											ssd1306_SetCursor(0,32);				
											
											if (master_array[menu_485_points_for_showing].master_type == 1 || master_array[menu_485_points_for_showing].master_type == 4)
											{
												snprintf(buffer, sizeof buffer, "%.02f", master_array[menu_485_points_for_showing].master_value);
											}
											else
											{
												snprintf(buffer, sizeof buffer, "%d", (int16_t) master_array[menu_485_points_for_showing].master_value);
											}
											
											ssd1306_WriteString(buffer,font_8x14,1);											
										}											

								}
								
								//ssd1306_UpdateScreen();				
							}

							if (menu_edit_settings_mode == 0) 
							for (uint8_t i = 0; i< REG_485_QTY; i++)
							{								
								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 0) //Значение регистра
								{
									ssd1306_Fill(0);
									
									ssd1306_SetCursor(0,0);																					
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);

									triangle_left(55,0);									
									if (i != REG_485_QTY-1) triangle_right(60,0);													
									triangle_down(58,43);									
									
									ssd1306_SetCursor(0,15);										
									strncpy(msg,"Значение регистра ", 18);						
									string_scroll_with_number(msg, 18, i);

									ssd1306_SetCursor(0,32);
									if (master_array[i].master_type == 1 || master_array[i].master_type == 4)
									{
										snprintf(buffer, sizeof buffer, "%.02f", master_array[i].master_value);
									}
									else
									{
										snprintf(buffer, sizeof buffer, "%d", (int16_t) master_array[i].master_value);
									}
									ssd1306_WriteString(buffer,font_8x14,1);
									
									menu_edit_mode = 0 ; //Запрещаем редактирование												 									
								}									
																
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 1) //Вкл/выкл опрос
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);		

									triangle_left(55,0);
									if (i != REG_485_QTY-1) triangle_right(60,0);
									triangle_up(58,38);
									triangle_down(58,43);
									
									ssd1306_SetCursor(0,15);										
									strncpy(msg,"Включить опрос регистра ", 24);						
									string_scroll_with_number(msg, 24, i);
									ssd1306_SetCursor(0,32);
									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode_int(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 0]);																																													
											//edit_mode_int8(&master_array[i].master_on);																																													
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%d", master_array[i].master_on);
										ssd1306_WriteString(buffer,font_8x14,1);										
									}	
									
								}									
								
								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 2) //Адрес устройства
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);	
									
									triangle_left(55,0);
									if (i != REG_485_QTY-1) triangle_right(60,0);
									triangle_up(58,38);
									triangle_down(58,43);
									
									ssd1306_SetCursor(0,15);									
									strncpy(msg,"Адрес устройства регистра ", 26);						
									string_scroll_with_number(msg, 26, i);
									
									ssd1306_SetCursor(0,32);
									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode_int(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 1]);																			
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%d", master_array[i].master_addr);
										ssd1306_WriteString(buffer,font_8x14,1);
									}	
								
								}	
								
								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 3) //Номер регистра
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);
									
									triangle_left(55,0);
									if (i != REG_485_QTY-1) triangle_right(60,0);
									triangle_up(58,38);
									triangle_down(58,43);									
									
									ssd1306_SetCursor(0,15);										
									strncpy(msg,"Адрес регистра ", 15);						
									string_scroll_with_number(msg, 15, i);

									ssd1306_SetCursor(0,32);									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode_int(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 2]);																			
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%d", master_array[i].master_numreg);
										ssd1306_WriteString(buffer,font_8x14,1);
									}	
						
								}	

								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 4) //Функциональный код
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);

									triangle_left(55,0);
									if (i != REG_485_QTY-1) triangle_right(60,0);
									triangle_up(58,38);
									triangle_down(58,43);
									
									ssd1306_SetCursor(0,15);										
									strncpy(msg,"Функциональный код регистра ", 28);						
									string_scroll_with_number(msg, 28, i);

									ssd1306_SetCursor(0,32);									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode_int(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 3]);																			
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%d", master_array[i].master_func);
										ssd1306_WriteString(buffer,font_8x14,1);
									}	
							
								}
								
								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 5) //Нижняя предупредительная уставка
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);

									triangle_left(55,0);
									if (i != REG_485_QTY-1) triangle_right(60,0);
									triangle_up(58,38);
									triangle_down(58,43);
									
									ssd1306_SetCursor(0,15);									
									strncpy(msg,"Нижняя предупредительная уставка регистра ", 42);						
									string_scroll_with_number(msg, 42, i);

									ssd1306_SetCursor(0,32);									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode(&master_array[i].low_master_warning_set);
										
											convert_float_and_swap(master_array[i].low_master_warning_set, &temp_buf[0]);	 
											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 12] = temp_buf[0];
											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 13] = temp_buf[1];																														
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%.01f", master_array[i].low_master_warning_set);
										ssd1306_WriteString(buffer,font_8x14,1);
									}	
						
								}
								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 6) //Нижняя аварийная уставка
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);

									triangle_left(55,0);
									if (i != REG_485_QTY-1) triangle_right(60,0);
									triangle_up(58,38);
									triangle_down(58,43);
									
									ssd1306_SetCursor(0,15);									
									strncpy(msg,"Нижняя аварийная уставка регистра ", 34);						
									string_scroll_with_number(msg, 34, i);

									ssd1306_SetCursor(0,32);									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode(&master_array[i].low_master_emergency_set);
										
											convert_float_and_swap(master_array[i].low_master_emergency_set, &temp_buf[0]);	 
											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 14] = temp_buf[0];
											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 15] = temp_buf[1];																														
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%.01f", master_array[i].low_master_emergency_set);
										ssd1306_WriteString(buffer,font_8x14,1);
									}									
								}											


								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 7) //Верхняя предупредительная уставка
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);

									triangle_left(55,0);
									if (i != REG_485_QTY-1) triangle_right(60,0);
									triangle_up(58,38);
									triangle_down(58,43);
									
									ssd1306_SetCursor(0,15);									
									strncpy(msg,"Верхняя предупредительная уставка регистра ", 43);						
									string_scroll_with_number(msg, 43, i);

									ssd1306_SetCursor(0,32);									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode(&master_array[i].master_warning_set);
										
											convert_float_and_swap(master_array[i].master_warning_set, &temp_buf[0]);	 
											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 16] = temp_buf[0];
											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 17] = temp_buf[1];																														
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%.01f", master_array[i].master_warning_set);
										ssd1306_WriteString(buffer,font_8x14,1);
									}	
						
								}		

								
								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 8) //Верхняя аварийная уставка
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);

									triangle_left(55,0);
									if (i != REG_485_QTY-1) triangle_right(60,0);
									triangle_up(58,38);
									triangle_down(58,43);
									
									ssd1306_SetCursor(0,15);									
									strncpy(msg,"Верхняя аварийная уставка регистра ", 35);						
									string_scroll_with_number(msg, 35, i);

									ssd1306_SetCursor(0,32);									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode(&master_array[i].master_emergency_set);
										
											convert_float_and_swap(master_array[i].master_emergency_set, &temp_buf[0]);	 
											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 18] = temp_buf[0];
											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 19] = temp_buf[1];																														
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%.01f", master_array[i].master_emergency_set);
										ssd1306_WriteString(buffer,font_8x14,1);
									}									
								}									

								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 9) //Коэф. А
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);
									
									triangle_left(55,0);
									if (i != REG_485_QTY-1) triangle_right(60,0);
									triangle_up(58,38);
									triangle_down(58,43);
									
									ssd1306_SetCursor(0,15);										
									strncpy(msg,"Коэффициент А регистра ", 23);						
									string_scroll_with_number(msg, 23, i);

									ssd1306_SetCursor(0,32);									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode(&master_array[i].master_coef_A);
										
											convert_float_and_swap(master_array[i].master_coef_A, &temp_buf[0]);	 
											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 6] = temp_buf[0];
											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 7] = temp_buf[1];																														
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%.05f", master_array[i].master_coef_A);
										ssd1306_WriteString(buffer,font_8x14,1);
									}									
								}		

								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 10) //Коэф. B
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);

									triangle_left(55,0);
									if (i != REG_485_QTY-1) triangle_right(60,0);
									triangle_up(58,38);
									triangle_down(58,43);
									
									ssd1306_SetCursor(0,15);										
									strncpy(msg,"Коэффициент В регистра ", 23);						
									string_scroll_with_number(msg, 23, i);

									ssd1306_SetCursor(0,32);									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode(&master_array[i].master_coef_B);
										
											convert_float_and_swap(master_array[i].master_coef_B, &temp_buf[0]);	 
											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 8] = temp_buf[0];
											settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 9] = temp_buf[1];																														
									}
									else //Нормальный режим
									{
										snprintf(buffer, sizeof buffer, "%.05f", master_array[i].master_coef_B);
										ssd1306_WriteString(buffer,font_8x14,1);
									}																
								}	

								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 11) //Тип данных
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);

									triangle_left(55,0);
									if (i != REG_485_QTY-1) triangle_right(60,0);
									triangle_up(58,38);
									triangle_down(58,43);
									
									ssd1306_SetCursor(0,15);										
									strncpy(msg,"Тип данных регистра ", 20);						
									string_scroll_with_number(msg, 20, i);

									ssd1306_SetCursor(0,32);									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode_int(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 4]);										
									}
									else //Нормальный режим
									{
											snprintf(buffer, sizeof buffer, "%d", master_array[i].master_type);
											ssd1306_WriteString(buffer,font_8x14,1);
									}																						
								}				

								
								if (menu_index_pointer == 3 && menu_horizontal == i+1 && menu_vertical == 12) //Таймаут
								{
									ssd1306_Fill(0);
									ssd1306_SetCursor(0,0);												
									snprintf(buffer, sizeof buffer, "485 %d", i);
									ssd1306_WriteString(buffer,font_8x14,1);
									
									triangle_left(55,0);
									if (i != REG_485_QTY-1) triangle_right(60,0);
									triangle_up(58,38);
																		
									ssd1306_SetCursor(0,15);										
									strncpy(msg,"Таймаут регистра ", 17);						
									string_scroll_with_number(msg, 17, i);

									ssd1306_SetCursor(0,32);									
									if (menu_edit_mode == 1) //Режим редактирования
									{											
											edit_mode_int(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 5]);										
									}
									else //Нормальный режим
									{
											snprintf(buffer, sizeof buffer, "%d", master_array[i].request_timeout);
											ssd1306_WriteString(buffer,font_8x14,1);
									}																						
								}									
								
							}							
							
								
							if (menu_index_pointer == 3 && menu_horizontal == 1 && menu_edit_settings_mode == 1) //Номер параметра для показа на гл. экране
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("485",font_8x14,1);												
								triangle_left(50,0);						
								triangle_right(55,0);							
								triangle_right(60,0);		
								
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Параметр на главном меню", 24);						
								string_scroll(msg, 24);
								
								ssd1306_SetCursor(0,32);			
								
								if (menu_edit_mode == 1) //Режим редактирования
								{											
											edit_mode_int(&menu_485_points_for_showing);										
								}
								else //Нормальный режим
								{
									snprintf(buffer, sizeof buffer, "%d", menu_485_points_for_showing);
									ssd1306_WriteString(buffer,font_8x14,1); 
								}										
												
							}
							
							if (menu_index_pointer == 3 && menu_horizontal == 2 && menu_edit_settings_mode == 1) //Скорость обмена
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("485",font_8x14,1);												
								triangle_left(50,0);														
																
								ssd1306_SetCursor(0,15);									
								strncpy(msg,"Скорость", 8);						
								string_scroll(msg, 8);
								
								ssd1306_SetCursor(0,32);			
								
								if (menu_edit_mode == 1) //Режим редактирования
								{
									edit_mode_from_list(&baud_rate_uart_3, (uint32_t*)&baudrate_array);
								}
								else 
								{
									snprintf(buffer, sizeof buffer, "%.00f", baud_rate_uart_3);			
									ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
								}								
												
							}								
							
							//ssd1306_UpdateScreen();
					}
					
//////////Реле

					if (menu_index_pointer == 4 && menu_horizontal == 0) //Состояние реле
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);																
						
						if (menu_edit_settings_mode == 0) 
						{
							triangle_right(60,0);																								
							triangle_up(58,38);
							triangle_down(58,43);							
						}
						else
						{
							triangle_right(55,0);
							triangle_right(60,0);																								
							triangle_up(58,38);
							triangle_down(58,43);
						}
						
						ssd1306_SetCursor(0,15);																									
						ssd1306_WriteString("Пред",font_8x15_RU,1);		
						ssd1306_WriteString(".",font_8x14,1);		
						ssd1306_SetCursor(42,16);
						snprintf(buffer, sizeof buffer, "%d", state_warning_relay);
						ssd1306_WriteString(buffer,font_8x14,1);							
						ssd1306_SetCursor(0,32);				
						ssd1306_WriteString("Авар",font_8x15_RU,1);		
						ssd1306_WriteString(".",font_8x14,1);		
						ssd1306_SetCursor(42,33);
						snprintf(buffer, sizeof buffer, "%d", state_emerg_relay);
						ssd1306_WriteString(buffer,font_8x14,1);							
						
						//ssd1306_UpdateScreen();				
						
						disable_up_down_button = 0;
					}							
					
					
					if (menu_index_pointer == 4 && menu_horizontal == 1 && menu_edit_settings_mode == 0) //Аттрибут события ICP, 4-20
					{

							ssd1306_Fill(0);												
							ssd1306_SetCursor(0,0);												
							ssd1306_WriteString("Реле",font_8x15_RU,1);									
							
							triangle_left(55,0);
							triangle_right(60,0);																								
							//triangle_up(58,38);
							//triangle_down(58,43);
							
							if (channel_485_ON == 1) 
							{
								ssd1306_SetCursor(0,15);						
								strncpy(msg,"Аттрибут события", 16);						
								string_scroll(msg, 16);
								ssd1306_WriteString(" 1",font_8x14,1);
								
								ssd1306_SetCursor(0,32);														
								snprintf(buffer, sizeof buffer, "0x%X", trigger_event_attribute);			
								ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
							}						
							else if (channel_485_ON == 2) // ЗСК
							{
								ssd1306_SetCursor(0,15);						
								strncpy(msg,"Бит состояния", 13);						
								string_scroll(msg, 13);
								ssd1306_WriteString("(0-15)",font_8x14,1);
								
								ssd1306_SetCursor(0,32);														
								snprintf(buffer, sizeof buffer, "0x%X", (uint16_t) trigger_485_ZSK);			
								ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим							
							}

							menu_edit_mode = 0 ; //Запрещаем редактирование						
							
							//ssd1306_UpdateScreen();			

							disable_up_down_button = 1;
					}						
					
					if (menu_index_pointer == 4 && menu_horizontal == 2 && menu_edit_settings_mode == 0) //Аттрибут события 485 пред. уставка
					{
							ssd1306_Fill(0);
							ssd1306_SetCursor(0,0);												
							ssd1306_WriteString("Реле",font_8x15_RU,1);			

							triangle_left(55,0);
							//triangle_right(60,0);																								
							//triangle_up(58,38);
							//triangle_down(58,43);
							
							if (channel_485_ON == 1) 
							{
								ssd1306_SetCursor(0,15);	
								
								strncpy(msg,"Аттрибут события", 16);						
								string_scroll(msg, 16);
								ssd1306_WriteString(" 2",font_8x14,1);
								
								ssd1306_SetCursor(0,32);														
								snprintf(buffer, sizeof buffer, "0x%X", trigger_485_event_attribute_warning);			
								ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
							}
							else if (channel_485_ON == 2) // ЗСК
							{
								ssd1306_SetCursor(0,15);						
								strncpy(msg,"Бит состояния", 13);						
								string_scroll(msg, 13);
								ssd1306_WriteString("(16-31)",font_8x14,1);
								
								ssd1306_SetCursor(0,32);														
								snprintf(buffer, sizeof buffer, "0x%X", (uint16_t) (trigger_485_ZSK >> 16));			
								ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим							
							}						

							menu_edit_mode = 0 ; //Запрещаем редактирование
							
							//ssd1306_UpdateScreen();		

							disable_up_down_button = 1;
					}						
					
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
					
					//Режим редактирования настроек реле
					if (menu_index_pointer == 4 && menu_horizontal == 1 && menu_edit_settings_mode == 1) //Режим работы реле
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			

						triangle_left(50,0);
						triangle_right(55,0);
						triangle_right(60,0);
						//triangle_up(58,38);
						//triangle_down(58,43);
						
						ssd1306_SetCursor(0,15);	
						ssd1306_WriteString("Режим",font_8x15_RU,1);		
						
						osDelay(200);
												
						ssd1306_SetCursor(0,32);						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&mode_relay);
							disable_up_down_button = 0;
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", mode_relay);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
							disable_up_down_button = 1;
						}
												
						//ssd1306_UpdateScreen();				
					}						


					if (menu_index_pointer == 4 && menu_horizontal == 2 && menu_edit_settings_mode == 1) //Задержка на срабатывание реле
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			

						triangle_left(50,0);
						triangle_right(55,0);
						triangle_right(60,0);
						//triangle_up(58,38);
						//triangle_down(58,43);						
						
						ssd1306_SetCursor(0,15);					
						strncpy(msg,"Задержка на срабатывание", 24);						
						string_scroll(msg, 24);
						
						ssd1306_SetCursor(0,32);						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&delay_relay);
							disable_up_down_button = 0;
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", delay_relay);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
							disable_up_down_button = 1;
						}
												
						//ssd1306_UpdateScreen();				
					}						
					
					
					if (menu_index_pointer == 4 && menu_horizontal == 3 && menu_edit_settings_mode == 1) //Задержка на выход из срабатывания реле
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			

						triangle_left(50,0);
						triangle_right(55,0);
						triangle_right(60,0);
						//triangle_up(58,38);
						//triangle_down(58,43);
						
						ssd1306_SetCursor(0,15);
						strncpy(msg,"Задержка на выход из срабатывания", 33);						
						string_scroll(msg, 33);
						
						ssd1306_SetCursor(0,32);						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&delay_relay_exit);
							disable_up_down_button = 0;
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", delay_relay_exit);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
							disable_up_down_button = 1;
						}
			
												
						//ssd1306_UpdateScreen();				
					}	
					

					if (menu_index_pointer == 4 && menu_horizontal == 4 && menu_edit_settings_mode == 1) //Тест работы реле
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			
						
						triangle_left(50,0);						
						//triangle_up(58,38);
						//triangle_down(58,43);
						
						ssd1306_SetCursor(0,15);	
						strncpy(msg,"Тест реле", 9);						
						string_scroll(msg, 9);
						
						ssd1306_SetCursor(0,32);																
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&test_relay);
							disable_up_down_button = 0;
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", test_relay);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
							disable_up_down_button = 1;
						}	
												
						//ssd1306_UpdateScreen();				
					}						
					
					
					
					
//////////Общие настройки	
					
					if (menu_index_pointer == 5 && menu_horizontal == 0) 
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
										
						triangle_right(60,0);
						triangle_up(58,38);
						triangle_down(58,43);						
						
						ssd1306_SetCursor(0,15);																									
						strncpy(msg,"Настройки", 9);						
						string_scroll(msg, 9);						

						//horizont_line(0,45);						
						
						//ssd1306_UpdateScreen();				
						
						disable_up_down_button = 0;
					}
					
					
					if (menu_index_pointer == 5 && menu_horizontal == 1) //Адрес устройства
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Настр",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
			
						triangle_left(55,0);
						triangle_right(60,0);
						//triangle_up(58,38);
						//triangle_down(58,43);						
						
						ssd1306_SetCursor(0,15);	
						ssd1306_WriteString("Адрес",font_8x15_RU,1);		
						osDelay(200);
						ssd1306_SetCursor(0,32);						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&slave_adr);
							disable_up_down_button = 0;
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", slave_adr);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
							disable_up_down_button = 1;
						}
												
						//ssd1306_UpdateScreen();				
					}	
												
					if (menu_index_pointer == 5 && menu_horizontal == 2) //Скорость обмена
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Настр",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					

						triangle_left(55,0);
						triangle_right(60,0);
						//triangle_up(58,38);
						//triangle_down(58,43);
						
						ssd1306_SetCursor(0,15);
						strncpy(msg,"Скорость", 8);						
						string_scroll(msg, 8);
						
						ssd1306_SetCursor(0,32);						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_from_list(&baud_rate_uart_2, (uint32_t*)&baudrate_array);
							disable_up_down_button = 0;
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%.00f", baud_rate_uart_2);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
							disable_up_down_button = 1;
						}
												
						//ssd1306_UpdateScreen();				
					}						
					
									
					
					if (menu_index_pointer == 5 && menu_horizontal == 3) //Время прогрева
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Настр",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						
						triangle_left(55,0);
						triangle_right(60,0);
						//triangle_up(58,38);
						//triangle_down(58,43);
						
						ssd1306_SetCursor(0,15);
						strncpy(msg,"Время прогрева", 14);						
						string_scroll(msg, 14);
						
						ssd1306_SetCursor(0,32);						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&warming_up);
							disable_up_down_button = 0;
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", warming_up);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
							disable_up_down_button = 1;
						}
												
						//ssd1306_UpdateScreen();				
					}		
					
					if (menu_index_pointer == 5 && menu_horizontal == 4) //Сброс настроек 
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Настр",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						
						triangle_left(55,0);						
						//triangle_up(58,38);
						//triangle_down(58,43);
						
						ssd1306_SetCursor(0,15);
						strncpy(msg,"Сброс настроек", 14);						
						string_scroll(msg, 14);
						
						ssd1306_SetCursor(0,32);						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&reset_to_default);
							disable_up_down_button = 0;
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", reset_to_default);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
							disable_up_down_button = 1;
						}

						//ssd1306_UpdateScreen();						
					}						



////////////Информация

					if (menu_index_pointer == 6 && menu_horizontal == 0) 
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);																		
												
						triangle_right(60,0);
						triangle_up(58,38);
												
						ssd1306_SetCursor(0,15);											
						strncpy(msg,"Информация", 10);						
						string_scroll(msg, 10);							
						//ssd1306_UpdateScreen();				
						disable_up_down_button = 0;
					}

					
					if (menu_index_pointer == 6 && menu_horizontal == 1) //Напряжение питания контроллера
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Инф",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						
						triangle_left(55,0);
						triangle_right(60,0);
						//triangle_up(58,38);
						
						ssd1306_SetCursor(0,15);
						strncpy(msg,"Напряжение питания", 18);						
						string_scroll(msg, 18);
						
						ssd1306_SetCursor(0,32);				
						snprintf(buffer, sizeof buffer, "%.01f", power_supply_voltage);				
						ssd1306_WriteString(buffer,font_8x14,1);
						//ssd1306_UpdateScreen();				
						
						disable_up_down_button = 1;
					}	
					
					if (menu_index_pointer == 6 && menu_horizontal == 2) //Версия ПО
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Инф",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					

						triangle_left(55,0);
						triangle_right(60,0);
						//triangle_up(58,38);						
						
						ssd1306_SetCursor(0,15);
						strncpy(msg,"Версия ПО", 9);						
						string_scroll(msg, 9);
						
						ssd1306_SetCursor(0,32);				
						snprintf(buffer, sizeof buffer, "%.02f", VERSION);				
						ssd1306_WriteString(buffer,font_8x14,1);
						//ssd1306_UpdateScreen();			

						disable_up_down_button = 1;						
					}
					
					
					if (menu_index_pointer == 6 && menu_horizontal == 3) //% ошибок timeout modbus master
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Инф",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						
						triangle_left(55,0);						
						//triangle_up(58,38);						
						
						ssd1306_SetCursor(0,15);
						ssd1306_WriteString("MMTE",font_8x14,1);	
						
						ssd1306_SetCursor(0,32);				
						snprintf(buffer, sizeof buffer, "%.01f", mb_master_timeout_error_percent);				
						ssd1306_WriteString(buffer,font_8x14,1);
						//ssd1306_UpdateScreen();				
						
						disable_up_down_button = 1;
					}					

							
					

//////////Конфигурация
					
					if (menu_index_pointer == 7 && menu_horizontal == 0 && config_mode == 1) 
					{
						ssd1306_Fill(0);											
						triangle_right(55,2);						
						ssd1306_SetCursor(0,15);																									

						strncpy(msg,"КОНФИГУРАЦИЯ", 12);						
						string_scroll(msg, 12);						

						//ssd1306_UpdateScreen();				
					}
					
					if (menu_index_pointer == 7 && menu_horizontal == 1) //Включаем канал ICP
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Конф",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);						
						triangle_right(55,2);				
						ssd1306_SetCursor(0,15);	
						ssd1306_WriteString("ICP",font_8x14,1);		
						ssd1306_SetCursor(0,32);				
						
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&channel_ICP_ON);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", channel_ICP_ON);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}
												
						//ssd1306_UpdateScreen();				
					}	
					
					
					if (menu_index_pointer == 7 && menu_horizontal == 2) //Включаем канал 4-20
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Конф",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);						
						triangle_right(55,2);				
						ssd1306_SetCursor(0,15);	
						ssd1306_WriteString("4-20",font_8x14,1);		
						ssd1306_SetCursor(0,32);				
						
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&channel_4_20_ON);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", channel_4_20_ON);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}
												
						//ssd1306_UpdateScreen();				
					}	
					
					
					if (menu_index_pointer == 7 && menu_horizontal == 3) //Включаем канал 485
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Конф",font_8x15_RU,1);																
						ssd1306_WriteString(".",font_8x14,1);					
						triangle_left(48,2);														
						ssd1306_SetCursor(0,15);	
						ssd1306_WriteString("485",font_8x14,1);		
						ssd1306_SetCursor(0,32);				
						
						
						if (menu_edit_mode == 1) //Режим редактирования
						{
							edit_mode_int(&channel_485_ON);
						}
						else 
						{
							snprintf(buffer, sizeof buffer, "%d", channel_485_ON);			
							ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим
						}
												
						//ssd1306_UpdateScreen();				
					}	
					
					
				//Рисуем на экранчике
				ssd1306_UpdateScreen();	
					
				//Инверсия переменной (для мигания меню в режиме редакции)	
				temp_stat_1 = !temp_stat_1;

			
			}
	
			osDelay(100);
  }
  /* USER CODE END Display_Task */
}

/* USER CODE BEGIN Header_Button_Task */
/**
* @brief Function implementing the myTask12 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Button_Task */
void Button_Task(void const * argument)
{
  /* USER CODE BEGIN Button_Task */
	uint8_t prev_state = 0;
  /* Infinite loop */
  for(;;)
  {

		//Лево	
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0)
		{
			button_left ++;
		}		
		else
		{
			if ( button_left > BUTTON_SENSE ) 
			{
				button_left_pressed_in = 1;				
				button_right_pressed_in = 0;
				button_up_pressed_in = 0;
				button_down_pressed_in = 0;
//				button_center_pressed_in_short = 0;
//				button_center_pressed_in_long = 0;
				button_center = 0;
				
				button_left = 0;
				
				temp_str = 0;
			}		
		}
		
		//Право
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)
		{
			button_right ++;		
		}		
		else
		{
			if ( button_right > BUTTON_SENSE ) 
			{
				button_left_pressed_in = 0;				
				button_right_pressed_in = 1;
				button_up_pressed_in = 0;
				button_down_pressed_in = 0;
//				button_center_pressed_in_short = 0;
//				button_center_pressed_in_long = 0;
				button_center = 0;
				
				button_right = 0;
				
				temp_str = 0;
			}	
		}
		
		//Вверх
		if (disable_up_down_button == 0)
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0)
		{
			button_up ++;
		}		
		else
		{
			if ( button_up > BUTTON_SENSE ) 
			{
				button_left_pressed_in = 0;				
				button_right_pressed_in = 0;
				button_up_pressed_in = 1;
				button_down_pressed_in = 0;
//				button_center_pressed_in_short = 0;
//				button_center_pressed_in_long = 0;
				button_center = 0;
				
				button_up = 0;
				temp_str = 0;
			}			
		}
		
		//Вниз
		if (disable_up_down_button == 0)
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0)
		{
			button_down ++;
		}		
		else
		{
			if ( button_down > BUTTON_SENSE ) 
			{
				button_left_pressed_in = 0;				
				button_right_pressed_in = 0;
				button_up_pressed_in = 0;
				button_down_pressed_in = 1;
//				button_center_pressed_in_short = 0;
//				button_center_pressed_in_long = 0;
				button_center = 0;
				
				button_down = 0;
				temp_str = 0;
			}			
		}
		
		//Центр
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 0)
		{
			button_center ++;	
			
			if ( button_center >= 100  && quit_relay_button == 0 ) 
			{
				button_left_pressed_in = 0;				
				button_right_pressed_in = 0;
				button_up_pressed_in = 0;
				button_down_pressed_in = 0;
				button_center_pressed_in_short = 0;
				button_center_pressed_in_long = 1;
								
				button_center = 0;
			}
		}
		else
		{
			if ( button_center > 2 && button_center < 100 && button_center_pressed_in_long == 0 && quit_relay_button == 0) 
			{				
					button_left_pressed_in = 0;				
					button_right_pressed_in = 0;
					button_up_pressed_in = 0;
					button_down_pressed_in = 0;
					button_center_pressed_in_short = 1;
					button_center_pressed_in_long = 0;
									
					button_center = 0;				
			}
			
			if (quit_relay_button == 1) button_center = 0;

		}	
	
		
    osDelay(10);
  }
  /* USER CODE END Button_Task */
}

/* USER CODE BEGIN Header_Modbus_Receive_Task */
/**
* @brief Function implementing the myTask13 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Modbus_Receive_Task */
void Modbus_Receive_Task(void const * argument)
{
  /* USER CODE BEGIN Modbus_Receive_Task */
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Modbus_Rx, portMAX_DELAY );					
						
		__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_IDLEF); 				
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
		
		HAL_UART_DMAStop(&huart2); 


		
		if (bootloader_state == 0x00)
		{
			HAL_UART_Receive_DMA(&huart2, receiveBuffer, 256);					
			
			if (receiveBuffer[1] == 0x62 && receiveBuffer[2] == 0x6F && receiveBuffer[3] == 0x6F && receiveBuffer[4] == 0x74)
			{
				JumpToApplication(BOOT_START_ADDRESS);
			}
			
			if (receiveBuffer[0] == 0x04 && receiveBuffer[1] == 0x08 && receiveBuffer[2] == 0x01 && receiveBuffer[7] == 0x01)
			{
				bootloader_state = 0x01;												
			}			
			else xSemaphoreGive( Semaphore_Modbus_Tx );
		}

		if (bootloader_state == 0x01)
		{
			boot_timer_counter = 0;					
			
			HAL_UART_Receive_DMA(&huart2, boot_receiveBuffer, 8);		
			
			if (worker_status == 0)
			{				
				xSemaphoreGive( Semaphore_Bootloader_Erase );
			}
			else xSemaphoreGive( Semaphore_Bootloader_Update	 );			
			
		}
    
  }
  /* USER CODE END Modbus_Receive_Task */
}

/* USER CODE BEGIN Header_Modbus_Transmit_Task */
/**
* @brief Function implementing the myTask14 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Modbus_Transmit_Task */
void Modbus_Transmit_Task(void const * argument)
{
  /* USER CODE BEGIN Modbus_Transmit_Task */
	uint16_t crc = 0;
	volatile uint16_t count_registers = 0;
	volatile uint16_t adr_of_registers = 0;
	volatile uint16_t recieve_calculated_crc = 0;
	volatile uint16_t recieve_actual_crc = 0;
	volatile uint16_t outer_register = 0;
	volatile uint16_t number_of_bytes_further = 0;
	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Modbus_Tx, portMAX_DELAY );
		
		//for (int i = 0; i < REG_COUNT*2+5; i++) transmitBuffer[i] = 0;
		
		if (receiveBuffer[0] == SLAVE_ADR)
		{		
				recieve_calculated_crc = crc16(receiveBuffer, 6);
				recieve_actual_crc = (receiveBuffer[7] << 8) + receiveBuffer[6];
				
				//Если 16 функция, другая длина пакета
				if (receiveBuffer[1] == 0x10) 
				{
					number_of_bytes_further = receiveBuffer[6]; 
					recieve_calculated_crc = crc16(receiveBuffer, 7 + number_of_bytes_further);
					recieve_actual_crc = (receiveBuffer[7 + number_of_bytes_further + 1] << 8) + receiveBuffer[7 + number_of_bytes_further];
				}
				
				//Проверяем crc
				if (recieve_calculated_crc == recieve_actual_crc) 
				{	
						transmitBuffer[0] = receiveBuffer[0]; //адрес устр-ва			
						transmitBuffer[1] = receiveBuffer[1]; //номер функции						
											
						adr_of_registers = (receiveBuffer[2] << 8) + receiveBuffer[3];//получаем адрес регистра				
						count_registers = (receiveBuffer[4] << 8) + receiveBuffer[5]; //получаем кол-во регистров из запроса
						outer_register = adr_of_registers + count_registers; //крайний регистр
					
						
						transmitBuffer[2] = count_registers*2; //количество байт	(в два раза больше чем регистров)	
					
					
//						//Проверяем номер регистра
//						if (adr_of_registers > REG_COUNT) 
//						{
//									if (transmitBuffer[1] == 0x3) transmitBuffer[1] = 0x83; //Function Code in Exception Response
//									if (transmitBuffer[1] == 0x4) transmitBuffer[1] = 0x84; //Function Code in Exception Response
//									
//									transmitBuffer[2] = 0x02; //Exception "Illegal Data Address"		
//									
//									crc = crc16(transmitBuffer, 3);
//							
//									transmitBuffer[3] = crc;
//									transmitBuffer[4] = crc >> 8;		 
//							
//									HAL_UART_Transmit_DMA(&huart2, transmitBuffer, 5);									
//						}					
						
						if (receiveBuffer[1] == 0x03 || receiveBuffer[1] == 0x04) //Holding Register (FC=03) or Input Register (FC=04)
						{		
									if (adr_of_registers < 125) //если кол-во регистров больше 125 (255 байт макс.), опрос идет несколькими запросами 
									{							
											for (uint16_t i=adr_of_registers, j=0; i < outer_register; i++, j++)
											{
												transmitBuffer[j*2+3] = settings[i] >> 8; //значение регистра Lo 		
												transmitBuffer[j*2+4] = settings[i] & 0x00FF; //значение регистра Hi		
											}
									
											crc = crc16(transmitBuffer, count_registers*2+3);				
									
											transmitBuffer[count_registers*2+3] = crc;
											transmitBuffer[count_registers*2+3+1] = crc >> 8;		
																				
												
											HAL_UART_Transmit_DMA(&huart2, transmitBuffer, count_registers*2+5);
									}
									else
									{
											if (adr_of_registers < 944) //Основная группа регистров
											{
												for (uint16_t i=0, j=0; i < count_registers; i++, j++)
												{
													transmitBuffer[j*2+3] = settings[adr_of_registers + i] >> 8; //значение регистра Lo 		
													transmitBuffer[j*2+4] = settings[adr_of_registers + i] & 0x00FF; //значение регистра Hi		
												}
											}
											
											if (adr_of_registers > 944 )//Зеркало (дублирование значений регистров 485 канала)
											{
												for (volatile uint16_t i=0, j=0; i < MIRROR_COUNT; i++, j++)
												{
													transmitBuffer[j*2+3] = mirror_values[i] >> 8; //значение регистра Lo 		
													transmitBuffer[j*2+4] = mirror_values[i] & 0x00FF; //значение регистра Hi		
												}												
											}
											
											if (adr_of_registers > 1080)	
											{
												for (uint16_t i=0, j=0; i < MIRROR_COUNT; i++, j++)
												{
													transmitBuffer[j*2+3] = 0; //значение регистра Lo 		
													transmitBuffer[j*2+4] = 0; //значение регистра Hi		
												}												
											}												
									
											crc = crc16(transmitBuffer, count_registers*2+3);				
									
											transmitBuffer[count_registers*2+3] = crc;
											transmitBuffer[count_registers*2+3+1] = crc >> 8;		
																				
															
											HAL_UART_Transmit_DMA(&huart2, transmitBuffer, count_registers*2+5);					
									}
						}							
						else if (receiveBuffer[1] == 0x06) //Preset Single Register (FC=06)
						{							
									if (adr_of_registers == 107)
									{
										settings[adr_of_registers] = (receiveBuffer[4] << 8) + receiveBuffer[5]; 										
									}
									else if (settings[107] == -7035) //Если изменяем регистры, то надо снять блокировку (Рег.108 = 0xE485)
									{
										settings[adr_of_registers] = (receiveBuffer[4] << 8) + receiveBuffer[5]; 										
									}

									transmitBuffer[2] = receiveBuffer[2];
									transmitBuffer[3] = receiveBuffer[3];
							
									transmitBuffer[4] = receiveBuffer[4];
									transmitBuffer[5] = receiveBuffer[5];
							
									crc = crc16(transmitBuffer, 6);				
							
									transmitBuffer[6] = crc;
									transmitBuffer[7] = crc >> 8;		
																		
							
									HAL_UART_Transmit_DMA(&huart2, transmitBuffer, 8);						
						}				
						else if (receiveBuffer[1] == 0x10) //Preset Multiply Registers (FC=16)
						{									
									if (adr_of_registers == 107)
									{
										settings[adr_of_registers] = (receiveBuffer[7] << 8) + receiveBuffer[8]; 										
										settings[adr_of_registers+1] = (receiveBuffer[9] << 8) + receiveBuffer[10];
									}
									else if (settings[107] == -7035) //Если изменяем регистры, то надо снять блокировку (Рег.108 = 0xE485)
									{
										for (uint16_t i=adr_of_registers, j=0; i < outer_register; i++, j=j+2)
										{
											settings[i] = (receiveBuffer[7 + j] << 8) + receiveBuffer[8 + j]; 										
											//settings[adr_of_registers+1] = (receiveBuffer[9] << 8) + receiveBuffer[10];										
										}
									}
									

									transmitBuffer[2] = receiveBuffer[2];//адрес первого регистра
									transmitBuffer[3] = receiveBuffer[3];
							
									transmitBuffer[4] = receiveBuffer[4];//кол-во регистров	
									transmitBuffer[5] = receiveBuffer[5];
								
							
									crc = crc16(transmitBuffer, 6);				
							
									transmitBuffer[6] = crc;
									transmitBuffer[7] = crc >> 8;		
									
							
									HAL_UART_Transmit_DMA(&huart2, transmitBuffer, 8);						
						}
						else
						{							
									transmitBuffer[1] = 0x81; //Function Code in Exception Response
									transmitBuffer[2] = 0x01; //Exception "Illegal function"			
									
									crc = crc16(transmitBuffer, 3);
							
									transmitBuffer[3] = crc;
									transmitBuffer[4] = crc >> 8;		 
							
									HAL_UART_Transmit_DMA(&huart2, transmitBuffer, 5);
						}					
						
				}
				
//				//Команда для перепрошивки
//				if (receiveBuffer[1] == 0x62 && receiveBuffer[2] == 0x6F && receiveBuffer[3] == 0x6F && receiveBuffer[4] == 0x74)
//				{					
//					
//					transmitBuffer[0] = 0x72;
//					transmitBuffer[1] = 0x65;
//					transmitBuffer[2] = 0x61;
//					transmitBuffer[3] = 0x64;
//					transmitBuffer[4] = 0x79;
//										
//					HAL_UART_Transmit_DMA(&huart2, transmitBuffer, 5);
//					
//					bootloader_state = 1;		
//					
//					JumpToApplication(BOOT_START_ADDRESS);
//					//rtc_write_backup_reg(1, bootloader_state);					
//					//NVIC_SystemReset();
//					
//					//receiveBuffer[1] = 0x00; boot_receiveBuffer[1] = 0x00;
//					
//				}
//				else bootloader_state = 0;
		}   
  }
  /* USER CODE END Modbus_Transmit_Task */
}

/* USER CODE BEGIN Header_Master_Modbus_Receive */
/**
* @brief Function implementing the myTask15 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Master_Modbus_Receive */
void Master_Modbus_Receive(void const * argument)
{
  /* USER CODE BEGIN Master_Modbus_Receive */
	uint16_t f_number = 0;
	volatile uint16_t byte_number = 0;
	uint16_t temp_data[2];
	volatile uint16_t calculated_crc = 0;
	volatile uint16_t actual_crc = 0;
	volatile float32_t temp;
	uint16_t rawValue = 0;
	
  /* Infinite loop */
  for(;;)
  {
		
		xSemaphoreTake( Semaphore_Master_Modbus_Rx, portMAX_DELAY );					
		
		__HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_IDLEF); 				
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		
		HAL_UART_DMAStop(&huart1); 
		
		HAL_UART_Receive_DMA(&huart1, master_receive_buffer, 9); 
		
		if (master_receive_buffer[0] == master_array[master_response_received_id].master_addr) //адрес
		{						
				f_number = master_receive_buffer[1]; //номер функции	
				byte_number = master_receive_buffer[2];//кол-во байт
			
				//считаем CRC
				calculated_crc = crc16(master_receive_buffer, 3 + byte_number);										
				actual_crc = master_receive_buffer[3 + byte_number];
				actual_crc += master_receive_buffer[3 + byte_number + 1] << 8;
				
				if (calculated_crc == actual_crc)
				{
						if (master_receive_buffer[1] == 0x03 || master_receive_buffer[1] == 0x04) //Holding Register (FC=03)
						{															
								if ( master_array[master_response_received_id].master_type == 0 ) //Тип данных, Dec
								{	
									master_array[master_response_received_id].master_value = (master_receive_buffer[3] << 8 ) + master_receive_buffer[4];
								}
								
								if ( master_array[master_response_received_id].master_type == 1 ) //Тип данных, Float (ABCD)
								{
									temp_data[0] = (master_receive_buffer[3] << 8 ) + master_receive_buffer[4];
									temp_data[1] = (master_receive_buffer[5] << 8 ) + master_receive_buffer[6];
									master_array[master_response_received_id].master_value = convert_hex_to_float(&temp_data[0], 0);
								}								
								
								
								if ( master_array[master_response_received_id].master_type == 2 || master_array[master_response_received_id].master_type == 5) //Тип данных, Int
								{									
									rawValue = (master_receive_buffer[3] << 8 ) + master_receive_buffer[4];
																		
									if ( rawValue >> 14 == 0 )
									{
										master_array[master_response_received_id].master_value = rawValue; 																			
									}
									else
									{										
										master_array[master_response_received_id].master_value = rawValue | ~((1 << 15) - 1);
									}
								}
								
								if ( master_array[master_response_received_id].master_type == 3 ) //Тип данных, Abs. int
								{									
									rawValue = (master_receive_buffer[3] << 8 ) + master_receive_buffer[4];
																		
									if ( rawValue >> 14 == 0 ) //Определяем знак
									{
										master_array[master_response_received_id].master_value = rawValue; 																			
									}
									else
									{										
										master_array[master_response_received_id].master_value = rawValue | ~((1 << 15) - 1);
										master_array[master_response_received_id].master_value = -master_array[master_response_received_id].master_value;
									}
								}

								
								if ( master_array[master_response_received_id].master_type == 4 ) //Тип данных, swFloat (swap words CDAB)
								{									
									temp_data[0] = (master_receive_buffer[5] << 8 ) + master_receive_buffer[6];
									temp_data[1] = (master_receive_buffer[3] << 8 ) + master_receive_buffer[4];
									
									master_array[master_response_received_id].master_value = convert_hex_to_float(&temp_data[0], 0);																		
								}									
						}
						
						//Применяем кооэф. к значению
						master_array[master_response_received_id].master_value = master_array[master_response_received_id].master_value * master_array[master_response_received_id].master_coef_A + master_array[master_response_received_id].master_coef_B;
						
						xTaskNotifyGive( xTask18 ); //Посылаем уведомление, если получен ответ 							
						
						//Устанавливаем признак "обрыва нет"
						break_sensor_485 = 0;
						//Обнуляем таймер обрыва датчика 485
						timer_485_counter = 0;
						
				}
				else 
				{
					mb_master_crc_error++;						
				}
				
				//Вычисляем проценты по CRC
				mb_master_crc_error_percent = mb_master_crc_error * 100.0 / (float32_t) mb_master_response;				
				if (mb_master_crc_error_percent < 0.1) mb_master_crc_error_percent = 0;				
				if (mb_master_crc_error_percent > 100.0) mb_master_crc_error_percent = 100;
				
				//Счетчик всех ответов
				mb_master_response++;			
		}

    
  }
  /* USER CODE END Master_Modbus_Receive */
}

/* USER CODE BEGIN Header_Master_Modbus_Transmit */
/**
* @brief Function implementing the myTask16 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Master_Modbus_Transmit */
void Master_Modbus_Transmit(void const * argument)
{
  /* USER CODE BEGIN Master_Modbus_Transmit */
	uint16_t crc = 0;
	TimeOut_t xTimeOut;
		
	
	
  /* Infinite loop */
  for(;;)
  {

		xTask18 = xTaskGetCurrentTaskHandle();	
		
		for(uint8_t i=0; i< REG_485_QTY; i++)
		{	
			
				if ( master_array[i].master_on == 1) //Если регистр выключен, то запрашиваем следующий		
				{				
						master_transmit_buffer[0] = master_array[i].master_addr;
						master_transmit_buffer[1] = master_array[i].master_func;
						master_transmit_buffer[2] = (master_array[i].master_numreg - 1) >> 8; 		//Смещение адреса, т.к. регистр номер 1 равно адресу 0.
						master_transmit_buffer[3] = (master_array[i].master_numreg - 1) & 0x00FF;
						master_transmit_buffer[4] = 0;			
						if (master_array[i].master_type == 0) master_transmit_buffer[5] = 1;
						else master_transmit_buffer[5] = 2;				
								
						crc = crc16(master_transmit_buffer, 6);				
								
						master_transmit_buffer[6] = crc;
						master_transmit_buffer[7] = crc >> 8;
						
						master_response_received_id = i;

					
						HAL_UART_Transmit_DMA(&huart1, master_transmit_buffer, 8);
					
						//Счетчик запросов
						mb_master_request++;

						
						//Фиксируем время для расчета таймаута					
						xTimeOutBefore = xTaskGetTickCount();			
						
						//Ждем уведомление о получении ответа, либо ошибка по таймауту
						ulTaskNotifyTake( pdTRUE, master_array[i].request_timeout ); 
						
						//Проверка таймаута
						xTotalTimeOutSuspended = xTaskGetTickCount() - xTimeOutBefore;					
						
						if ( xTotalTimeOutSuspended >= master_array[i].request_timeout ) 
						{
							reg_lost_packet[i] += 1;
							mb_master_timeout_error++;						
							master_array[i].master_value = 0;
						}
						
						//Вычисляем проценты по таймауту
						mb_master_timeout_error_percent = (float32_t) mb_master_timeout_error * 100.0 / mb_master_request; 		
						if (mb_master_timeout_error_percent < 0) mb_master_timeout_error_percent = 0;
						if (mb_master_timeout_error_percent > 100) mb_master_timeout_error_percent = 100;
						
						osDelay(3); //Silent time/delay between modbus request (must 3.5 char)
				}				
		}
			
		//Период опроса всех регистров, мс
		osDelay(mb_master_timeout);
	    
  }
  /* USER CODE END Master_Modbus_Transmit */
}

/* USER CODE BEGIN Header_Data_Storage_Task */
/**
* @brief Function implementing the myTask17 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Data_Storage_Task */
void Data_Storage_Task(void const * argument)
{
  /* USER CODE BEGIN Data_Storage_Task */
	uint16_t temp[2];
	volatile uint8_t st_flash = 0;
	volatile float32_t y = 0;
	
  /* Infinite loop */
  for(;;)
  {
		
		//Смещение на -1 (т.е. 1й регистр == settings[0])
		
		convert_float_and_swap(icp_voltage, &temp[0]);				
		settings[0] = temp[0];
		settings[1] = temp[1];

		settings[10] = break_sensor_icp;
		
		convert_float_and_swap(rms_acceleration_icp, &temp[0]);		
		settings[22] = temp[0];
		settings[23] = temp[1];			
		convert_float_and_swap(rms_velocity_icp, &temp[0]);
		settings[24] = temp[0];
		settings[25] = temp[1];	
		convert_float_and_swap(rms_displacement_icp, &temp[0]);
		settings[26] = temp[0];
		settings[27] = temp[1];
		
		settings[30] = trigger_485_ZSK; 				//Младшие биты
		settings[31] = trigger_485_ZSK >> 16;		//Старшие биты
		settings[32] = trigger_485_ZSK_percent;
		
		
		convert_float_and_swap(mean_4_20, &temp[0]);		
		settings[36] = temp[0];
		settings[37] = temp[1];
		
		settings[46] = break_sensor_420;
		
		convert_float_and_swap(calculated_value_4_20, &temp[0]);		
		settings[55] = temp[0];
		settings[56] = temp[1];

		convert_float_and_swap(max_4_20, &temp[0]);		
		settings[58] = temp[0];
		settings[59] = temp[1];
		convert_float_and_swap(max_4_20 - min_4_20, &temp[0]);		
		settings[60] = temp[0];
		settings[61] = temp[1];

		settings[64] = menu_485_points_for_showing;

		settings[70] = trigger_485_event_attribute_warning;
		settings[71] = trigger_485_event_attribute_emerg;
		settings[73] = break_sensor_485; 
		
		
		convert_float_and_swap(mb_master_crc_error_percent, &temp[0]);	
		settings[74] = temp[0];
		settings[75] = temp[1];
		convert_float_and_swap(mb_master_timeout_error_percent, &temp[0]);	
		settings[76] = temp[0];
		settings[77] = temp[1];	
		
		if ( mb_master_request > mb_master_response )
		{
			convert_float_and_swap((mb_master_request - mb_master_response), &temp[0]);				
		}
		else 
		{
			convert_float_and_swap(0, &temp[0]);				
		}
		
		settings[78] = temp[0];
		settings[79] = temp[1];	

		settings[80] = warning_relay_counter; 
		settings[81] = emerg_relay_counter; 
		settings[82] = state_warning_relay;
		settings[83] = state_emerg_relay;
		
		settings[87] = trigger_event_attribute;

		convert_float_and_swap(power_supply_voltage, &temp[0]);		
		settings[98] = temp[0];
		settings[99] = temp[1];

		convert_float_and_swap(cpu_float, &temp[0]);		
		settings[103] = temp[0];
		settings[104] = temp[1];
		
		convert_float_and_swap(VERSION, &temp[0]);
		settings[105] = temp[0];
		settings[106] = temp[1];

		settings[120] = hart_value;
		
		convert_float_and_swap(max_acceleration_icp, &temp[0]);	
		settings[122] = temp[0];
		settings[123] = temp[1];
		convert_float_and_swap(max_velocity_icp, &temp[0]);	
		settings[124] = temp[0];
		settings[125] = temp[1];
		convert_float_and_swap(max_displacement_icp, &temp[0]);	
		settings[126] = temp[0];
		settings[127] = temp[1];				
		convert_float_and_swap(max_acceleration_icp - min_acceleration_icp, &temp[0]);	
		settings[128] = temp[0];
		settings[129] = temp[1];
		convert_float_and_swap(max_velocity_icp - min_velocity_icp, &temp[0]);	
		settings[130] = temp[0];
		settings[131] = temp[1];
		convert_float_and_swap(max_displacement_icp - min_displacement_icp, &temp[0]);	
		settings[132] = temp[0];
		settings[133] = temp[1];

		

		convert_float_and_swap(turnover_count_60s, &temp[0]);	
		settings[136] = temp[0];
		settings[137] = temp[1];


	
		if (menu_edit_mode == 0)
		for (uint16_t i = 0; i < REG_485_QTY; i++)
		{			
						
				if (channel_485_ON == 2) //Специальный режим работы для системы ЗСК	
				if(MOVING_AVERAGE == 1) //Расчитываем скользящее среднее и перезаписываем значение с учетом усреднения (ЗСК)		
				if (i < 15) //Усредняем только вибропараметры
				{												
						average_result = 0.0;																	
					
						//Сдвигаем массив для записи нового элемента
						for (int y = 1; y < size_moving_average_ZSK; y++)				
						{
								zsk_average_array[i][y-1] = zsk_average_array[i][y];
						}						
						
						//Записываем новое значение 
						zsk_average_array[i][size_moving_average_ZSK - 1] = master_array[i].master_value;
						
						//Расчитываем среднее
						for (int j = 0; j < size_moving_average_ZSK; j++)				
						{
								average_result += zsk_average_array[i][j] / size_moving_average_ZSK;
						}
						
						master_array[i].master_value = average_result;
				}			
			
			
				master_array[i].master_on = settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 0];
				master_array[i].master_addr = settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 1];
				master_array[i].master_numreg = settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 2];
				master_array[i].master_func = settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 3];
				master_array[i].master_type = settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 4];
				master_array[i].request_timeout = settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 5];		
				
				master_array[i].master_coef_A = convert_hex_to_float(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 6], 0); 
				master_array[i].master_coef_B = convert_hex_to_float(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 8], 0);
								 
			
				if (master_array[i].master_type == 0) //Тип, dec
				{
					settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 10] = master_array[i].master_value; 
					
					//Копируем значения в зеркало
					mirror_values[i*2] = master_array[i].master_value;
				}
				
								
				if (master_array[i].master_type == 1 || master_array[i].master_type == 4) //Тип, float 
				{					
					convert_float_and_swap(master_array[i].master_value, &temp[0]);	 
					settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 10] = temp[0];
					settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 11] = temp[1];		
					
					//Копируем значения в зеркало
					mirror_values[i*2] = temp[0];
					mirror_values[i*2+1] = temp[1];
				}
				
				if (master_array[i].master_type == 2 || master_array[i].master_type == 3) //Тип, int
				{
					settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 10] = (int16_t) master_array[i].master_value; 
					
					//Копируем значения в зеркало
					mirror_values[i*2] = (int16_t) master_array[i].master_value;
				}

				master_array[i].low_master_warning_set = convert_hex_to_float(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 12], 0);	
				master_array[i].low_master_emergency_set = convert_hex_to_float(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 14], 0);	

				master_array[i].master_warning_set = convert_hex_to_float(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 16], 0);	
				master_array[i].master_emergency_set = convert_hex_to_float(&settings[REG_485_START_ADDR + STRUCTURE_SIZE*i + 18], 0);	

				
				
		}


		//Применение/запись настроек + запись метрологических коэф.
		if (settings[107] == 481) //0x1E1 int16 Big Endian (AB)
		{		
									
			settings[107] = 0x00;
			
			taskENTER_CRITICAL(); 						
			st_flash = write_registers_to_flash(settings);						
			read_init_settings();			
			taskEXIT_CRITICAL(); 					
			
			init_menu(0);
			FilterInit();
								
			//NVIC_SystemReset();			
		}

		
		
		//Применение/запись настроек без метрологии 
		if (settings[107] == -21555) //0xABCD int16 Big Endian (AB)
		{		
						
			settings[107] = 0x00;			

			convert_float_and_swap(icp_coef_K, &temp[0]);			
			settings[15] = temp[0];  
			settings[16] = temp[1];			
			convert_float_and_swap(icp_coef_B, &temp[0]);			
			settings[17] = temp[0];  
			settings[18] = temp[1];  
			convert_float_and_swap(FILTER_MODE, &temp[0]);			
			settings[19] = temp[0];  
			settings[20] = temp[1];
			
			convert_float_and_swap(coef_ampl_420, &temp[0]);			
			settings[51] = temp[0];
			settings[52] = temp[1]; 			
			convert_float_and_swap(coef_offset_420, &temp[0]);			
			settings[53] = temp[0];
			settings[54] = temp[1]; 
			
			convert_float_and_swap(out_4_20_coef_K, &temp[0]);
			settings[90] = temp[0];			
			settings[91] = temp[1];				
			convert_float_and_swap(out_4_20_coef_B, &temp[0]);
			settings[92] = temp[0];							
			settings[93] = temp[1];				
			
			taskENTER_CRITICAL(); 						
			st_flash = write_registers_to_flash(settings);						
			read_init_settings();			
			taskEXIT_CRITICAL(); 					
			
			init_menu(0);
			FilterInit();
			
			//xSemaphoreGive( Mutex_Setting );			
			//NVIC_SystemReset();			
		}
		
		//Сброс настроек
		if (settings[108] == -9030 || (menu_edit_mode == 0 && reset_to_default == 1)) //0xDCBA int16 Big Endian (AB)
		{
			settings[108] = 0x0;
			
			for(uint16_t i=0; i< REG_COUNT; i++) 
			{
				if ( 	i == 15 || i == 17 || 
							i == 51 || i == 53 ||
							i == 90 || i == 92 )
				{				
					settings[i] = settings[i];			
				}
				else settings[i] = 0;	
			}


			settings[100] = 10; 		
			
			settings[109] = 20000; 						
			
			convert_float_and_swap(22, &temp[0]);	
			settings[110] = temp[0];
			settings[111] = temp[1];		
			
			convert_float_and_swap(26, &temp[0]);	
			settings[112] = temp[0];
			settings[113] = temp[1];		
			
			convert_float_and_swap(115200, &temp[0]);	
			settings[65] = temp[0];
			settings[66] = temp[1];			
			settings[68] = temp[0];
			settings[69] = temp[1];
			settings[101] = temp[0];
			settings[102] = temp[1];		
	
			settings[119] = 100;		


			st_flash = write_registers_to_flash(settings);	

			NVIC_SystemReset();	
		}
		
		
    osDelay(10);
  }
  /* USER CODE END Data_Storage_Task */
}

/* USER CODE BEGIN Header_TriggerLogic_Task */
/**
* @brief Function implementing the myTask18 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TriggerLogic_Task */
void TriggerLogic_Task(void const * argument)
{
  /* USER CODE BEGIN TriggerLogic_Task */

	
	osDelay(warming_up);
	warming_flag = 0;
	
	
	
  /* Infinite loop */
  for(;;)
  {
		
		//Обнуляем состояние только в режиме работы "без памяти"
		if (mode_relay == 0)
		{
			state_warning_relay = 0;
			state_emerg_relay = 0;
		}
		
		
		if (warming_flag == 0)
		{			
				//Источник сигнала ICP
				if (channel_ICP_ON == 1)
				{
						//Предупр. реле
						if ( rms_velocity_icp >= hi_warning_icp || break_sensor_icp == 1 )								
						{
							flag_delay_relay_1_icp = 1; //Запускаем таймер
							
							if (relay_permission_1_icp == 1) //Если разрешение получено, то работаем
							{
								state_warning_relay = 1;						
								trigger_event_attribute |= (1<<15);					
								flag_for_delay_relay_exit = 1;			
							
								xSemaphoreGive( Semaphore_Relay_1 );
							}								
						}						
						else if ( rms_velocity_icp < hi_warning_icp )
						{							
							if (mode_relay == 0) trigger_event_attribute &= ~(1<<15);	

							timer_delay_relay_1_icp = 0;
							relay_permission_1_icp = 0;	
							flag_delay_relay_1_icp = 0; 							
						}
						
						//Авар. реле
						if ( rms_velocity_icp >= hi_emerg_icp || break_sensor_icp == 1 ) 
						{								
							flag_delay_relay_2_icp = 1; //Запускаем таймер
							
							if (relay_permission_2_icp == 1) //Если разрешение получено, то работаем
							{
								state_emerg_relay = 1;														
								trigger_event_attribute |= (1<<14);
								flag_for_delay_relay_exit = 1;														
							
								xSemaphoreGive( Semaphore_Relay_2 );
							}
							
						}
						else if ( rms_velocity_icp < hi_emerg_icp )
						{							
							if (mode_relay == 0) trigger_event_attribute &= ~(1<<14);
							
							timer_delay_relay_2_icp = 0;
							relay_permission_2_icp = 0;	
							flag_delay_relay_2_icp = 0; 
						}
				}
				
				//Источник сигнала 4-20
				if (channel_4_20_ON == 1)
				{		
						//Предупредительная
						if ( calculated_value_4_20 >= hi_warning_420 || calculated_value_4_20 <= lo_warning_420 || break_sensor_420 == 1 ) 
						{							
							
							flag_delay_relay_1_4_20 = 1; //Запускаем таймер
							
							if (relay_permission_1_4_20 == 1) //Если разрешение получено, то работаем
							{	
								state_warning_relay = 1;			
								trigger_event_attribute |= (1<<13);
								flag_for_delay_relay_exit = 1;														
								xSemaphoreGive( Semaphore_Relay_1 );							
							}
							
						}						
						else 
						{
							if ( mean_4_20 > lo_warning_420 && mean_4_20 < hi_warning_420 ) //Если сигнал ниже предупр. уставки
							{							
								if (mode_relay == 0) trigger_event_attribute &= ~(1<<13);
								
								timer_delay_relay_1_4_20 = 0;
								relay_permission_1_4_20 = 0;	
								flag_delay_relay_1_4_20 = 0; 								
							}													
						}
						
						
						//Аварийная
						if ( calculated_value_4_20 <= lo_emerg_420 || calculated_value_4_20 >= hi_emerg_420 || break_sensor_420 == 1 ) 
						{							
							flag_delay_relay_2_4_20 = 1; //Запускаем таймер

							if (relay_permission_2_4_20 == 1) //Если разрешение получено, то работаем
							{													
								state_emerg_relay = 1;
								trigger_event_attribute |= (1<<12);				
								flag_for_delay_relay_exit = 1;
								xSemaphoreGive( Semaphore_Relay_2 );							
							}
						}
						else if ( calculated_value_4_20 > lo_emerg_420 && calculated_value_4_20 < hi_emerg_420 ) 
						{							
							if (mode_relay == 0) trigger_event_attribute &= ~(1<<12);		

							if ( calculated_value_4_20 > lo_emerg_420 && calculated_value_4_20 < hi_emerg_420 )
							{
								timer_delay_relay_2_4_20 = 0;
								relay_permission_2_4_20 = 0;	
								flag_delay_relay_2_4_20 = 0; 
							}							
						}
				}
				
				//Источник сигнала 485 (Modbus)
				if (channel_485_ON == 1)
				{		

						for (uint8_t i = 0; i< REG_485_QTY; i++)
						{
								if (master_array[i].master_on == 1)
								{			
										//Предупредительная уставка
										if (master_array[i].master_value >= master_array[i].master_warning_set || master_array[i].master_value <= master_array[i].low_master_warning_set || break_sensor_485 == 1) 
										{
											
											master_delay_relay_array[i].flag_delay_relay_1 = 1;
											
											if (master_delay_relay_array[i].relay_permission_1 == 1)
											{
												trigger_485_event_attribute_warning |= (1<<(15-i));								
												state_warning_relay = 1;
												flag_for_delay_relay_exit = 1;							
												xSemaphoreGive( Semaphore_Relay_1 );							
											}
										}	
										else if (master_array[i].master_value < master_array[i].master_warning_set || master_array[i].master_value > master_array[i].low_master_warning_set) 						
										{
											if (mode_relay == 0) trigger_485_event_attribute_warning &= ~(1<<(15-i));								

											master_delay_relay_array[i].timer_delay_relay_1 = 0;
											master_delay_relay_array[i].relay_permission_1 = 0;	
											master_delay_relay_array[i].flag_delay_relay_1 = 0;											
										}
										
										//Аварийная уставка
										if (master_array[i].master_value >= master_array[i].master_emergency_set || master_array[i].master_value <= master_array[i].low_master_emergency_set || break_sensor_485 == 1) 
										{											
											master_delay_relay_array[i].flag_delay_relay_2 = 1;
											
											if (master_delay_relay_array[i].relay_permission_2 == 1)
											{
												trigger_485_event_attribute_emerg |= (1<<(15-i));																			
												state_emerg_relay = 1;
												flag_for_delay_relay_exit = 1;							
												xSemaphoreGive( Semaphore_Relay_2 );							
											}
										}	
										else if (master_array[i].master_value < master_array[i].master_emergency_set || master_array[i].master_value > master_array[i].low_master_emergency_set)						
										{
											if (mode_relay == 0) trigger_485_event_attribute_emerg &= ~(1<<(15-i));		

											master_delay_relay_array[i].timer_delay_relay_2 = 0;
											master_delay_relay_array[i].relay_permission_2 = 0;	
											master_delay_relay_array[i].flag_delay_relay_2 = 0; 											
										}										
										

								}
						}		
						
				}
				
				
				if (channel_485_ON == 2) //Специальный режим работы для системы ЗСК
				{
					
						for (uint8_t i = 0; i < ZSK_REG_485_QTY; i++)
						{
								if (master_array[i].master_on == 1)
								{			
									
										if ((i >= 0) && (i < 15)) //Регистры с вибропараметрами
										{
											
												//Нижняя предупредительная уставка
												if (master_array[i].master_value >= master_array[i].low_master_warning_set) 
												{											
													
													if (i == 0 || i == 1 || i == 2) trigger_485_ZSK |= (1<<0);
													if (i == 3 || i == 4 || i == 5) trigger_485_ZSK |= (1<<1);											
													if (i == 6 || i == 7 || i == 8) trigger_485_ZSK |= (1<<2);											
													if (i == 9 || i == 10 || i == 11) trigger_485_ZSK |= (1<<3);
													if (i == 12 || i == 13 || i == 14) trigger_485_ZSK |= (1<<4);														

												}												
												
												//Предупредительная уставка											
												if (master_array[i].master_value >= master_array[i].master_warning_set) 
												{
													
														master_delay_relay_array[i].flag_delay_relay_1 = 1;
														
														if (master_delay_relay_array[i].relay_permission_1 == 1)
														{															
															if (i == 0 || i == 1 || i == 2) trigger_485_ZSK |= (1<<5);
															if (i == 3 || i == 4 || i == 5) trigger_485_ZSK |= (1<<6);
															if (i == 6 || i == 7 || i == 8) trigger_485_ZSK |= (1<<7);
															if (i == 9 || i == 10 || i == 11) trigger_485_ZSK |= (1<<8);
															if (i == 12 || i == 13 || i == 14) trigger_485_ZSK |= (1<<9);																															
															
															state_warning_relay = 1;
															flag_for_delay_relay_exit = 1;							
															xSemaphoreGive( Semaphore_Relay_1 );																
														}
												}	
												else if (master_array[i].master_value < master_array[i].master_warning_set) 						
												{
														master_delay_relay_array[i].timer_delay_relay_1 = 0;
														master_delay_relay_array[i].relay_permission_1 = 0;	
														master_delay_relay_array[i].flag_delay_relay_1 = 0;											
												}
										
										
										
												//Аварийная уставка
												if (master_array[i].master_value >= master_array[i].master_emergency_set) 
												{											
													master_delay_relay_array[i].flag_delay_relay_2 = 1;
													
													if (master_delay_relay_array[i].relay_permission_2 == 1)
													{
														
															if (i == 0) trigger_485_ZSK |= (1<<10);
															if (i == 1) trigger_485_ZSK |= (1<<11);
															if (i == 2) trigger_485_ZSK |= (1<<12);
															
															if (i == 3) trigger_485_ZSK |= (1<<13);
															if (i == 4) trigger_485_ZSK |= (1<<14);
															if (i == 5) trigger_485_ZSK |= (1<<15);
															
															if (i == 6) trigger_485_ZSK |= (1<<16);
															if (i == 7) trigger_485_ZSK |= (1<<17);
															if (i == 8) trigger_485_ZSK |= (1<<18);
															
															if (i == 9) trigger_485_ZSK |= (1<<19);
															if (i == 10) trigger_485_ZSK |= (1<<20);
															if (i == 11) trigger_485_ZSK |= (1<<21);												
															
															if (i == 12) trigger_485_ZSK |= (1<<22);
															if (i == 13) trigger_485_ZSK |= (1<<23);
															if (i == 14) trigger_485_ZSK |= (1<<24);																								

															
															//Проверка на срабатывание по осям	
															x_axis = (((trigger_485_ZSK & (1<<10)) != 0) ||
															((trigger_485_ZSK & (1<<13)) != 0) ||
															((trigger_485_ZSK & (1<<16)) != 0) ||
															((trigger_485_ZSK & (1<<19)) != 0) ||
															((trigger_485_ZSK & (1<<22)) != 0));
															
															y_axis = (((trigger_485_ZSK & (1<<11)) != 0) ||
															((trigger_485_ZSK & (1<<14)) != 0) ||
															((trigger_485_ZSK & (1<<17)) != 0) ||
															((trigger_485_ZSK & (1<<20)) != 0) ||
															((trigger_485_ZSK & (1<<23)) != 0));
															
															z_axis = (((trigger_485_ZSK & (1<<12)) != 0) ||
															((trigger_485_ZSK & (1<<15)) != 0) ||
															((trigger_485_ZSK & (1<<18)) != 0) ||
															((trigger_485_ZSK & (1<<21)) != 0) ||
															((trigger_485_ZSK & (1<<24)) != 0));												
															

															if ( (x_axis & y_axis) ||  (x_axis & z_axis) || (y_axis & z_axis) )												
															{
																state_emerg_relay = 1;
																flag_for_delay_relay_exit = 1;
																xSemaphoreGive( Semaphore_Relay_2 );							
															}														
													}
												}	
												else if (master_array[i].master_value < master_array[i].master_emergency_set)						
												{
														master_delay_relay_array[i].timer_delay_relay_2 = 0;
														master_delay_relay_array[i].relay_permission_2 = 0;	
														master_delay_relay_array[i].flag_delay_relay_2 = 0; 											
												}

										}
										else if (i >= 15) //Регистры с углами
										{
												//Предупредительная уставка											
												if (master_array[i].master_value >= master_array[i].master_warning_set || master_array[i].master_value <= master_array[i].low_master_warning_set) 
												{
													
														master_delay_relay_array[i].flag_delay_relay_1 = 1;
														
														if (master_delay_relay_array[i].relay_permission_1 == 1)
														{
											
															if (i == 15 || i == 16 || i == 17) trigger_485_ZSK |= (1<<25);																	
															
															state_warning_relay = 1;
															flag_for_delay_relay_exit = 1;							
															xSemaphoreGive( Semaphore_Relay_1 );							
														}
												}	
												else if (master_array[i].master_value < master_array[i].master_warning_set) 						
												{
														master_delay_relay_array[i].timer_delay_relay_1 = 0;
														master_delay_relay_array[i].relay_permission_1 = 0;	
														master_delay_relay_array[i].flag_delay_relay_1 = 0;											
												}
										
										
										
												//Аварийная уставка
												if (master_array[i].master_value >= master_array[i].master_emergency_set || master_array[i].master_value <= master_array[i].low_master_emergency_set) 
												{											
													master_delay_relay_array[i].flag_delay_relay_2 = 1;
													
													if (master_delay_relay_array[i].relay_permission_2 == 1)
													{
															
															if (i == 15) trigger_485_ZSK |= (1<<26);
															if (i == 16) trigger_485_ZSK |= (1<<27);
															if (i == 17) trigger_485_ZSK |= (1<<28);																								

															
															if ( ((trigger_485_ZSK & (1<<26)) != 0) || ((trigger_485_ZSK & (1<<27)) != 0) || ((trigger_485_ZSK & (1<<28)) != 0)  )
															{
																state_emerg_relay = 1;
																flag_for_delay_relay_exit = 1;	
																xSemaphoreGive( Semaphore_Relay_2 );							
															}
													}
												}	
												else if (master_array[i].master_value < master_array[i].master_emergency_set)						
												{
														master_delay_relay_array[i].timer_delay_relay_2 = 0;
														master_delay_relay_array[i].relay_permission_2 = 0;	
														master_delay_relay_array[i].flag_delay_relay_2 = 0; 											
												}											
										}										
								}
						}							
						
						
						
						//Обработка регистра состояния оборудования (расчет процента, биты -> проценты)						

						for (int i = 0; i < ZSK_REG_485_QTY; i++) //Раскидываем биты по массиву
						{
							ZSK_trigger_array[i] = trigger_485_ZSK & (1<<i);
						}
						
						for(int i = 0; i < ZSK_REG_485_QTY; i++) 
						{
							if (i >= 0 && i < 5) 
							{
								if ((ZSK_trigger_array_previous[i] == 0) && (ZSK_trigger_array[i] != 0)) 
								{
									trigger_485_ZSK_percent += 5;								
								}
							}
							
							if (i >= 5 && i < 10) 
							{								
								if ((ZSK_trigger_array_previous[i] == 0) && (ZSK_trigger_array[i] != 0)) 								
								{
									if (trigger_485_ZSK_percent < 50) trigger_485_ZSK_percent = 50; //Базис								
									
									trigger_485_ZSK_percent += 5;								
								}
							}
							
							if (i >= 10 && i < 26) 
							{
								if ((ZSK_trigger_array_previous[i] == 0) && (ZSK_trigger_array[i] != 0)) 
								{
									trigger_485_ZSK_percent = 90;								
								}
							}							
							
							if (i >= 26 && i < 29)
							{
								if ((ZSK_trigger_array_previous[i] == 0) && (ZSK_trigger_array[i] != 0)) 
								{
									trigger_485_ZSK_percent = 100;
								}
							}							
						}
						
						if (trigger_485_ZSK_percent < 0)  trigger_485_ZSK_percent = 0;
						else if (trigger_485_ZSK_percent > 100) trigger_485_ZSK_percent = 100;
						
						if ( (x_axis & y_axis) ||  (x_axis & z_axis) || (y_axis & z_axis) )	
						{
							trigger_485_ZSK_percent = 100;
						}
						
						
						//Если было событие, сохраняем регистр состояния на flash
						if (trigger_485_ZSK_percent_prev != trigger_485_ZSK_percent) 
						{
							write_reg_flash(104, trigger_485_ZSK, 1);
							trigger_485_ZSK_percent_prev = trigger_485_ZSK_percent;
						}
						else trigger_485_ZSK_percent_prev = trigger_485_ZSK_percent;
						
						//Фиксируем текущее состояние, для того чтоб не было одновременных нескольких срабатываний при подъеме бита
						memcpy(ZSK_trigger_array_previous, ZSK_trigger_array, sizeof(ZSK_trigger_array));
						
						
						//Обрыв датчика
						if(break_sensor_485 == 1) 
						{
							state_emerg_relay = 1;							
							xSemaphoreGive( Semaphore_Relay_2 );							
						}
						
				}					
				
				
				if (mode_relay == 0)
				{
						//Сброс предупр. реле 
						if (state_warning_relay == 0 && relay_permission_1_4_20 == 0)  
						{								
							xSemaphoreGive( Semaphore_Relay_1 );							
						}
						
						//Сброс авар. реле 
						if (state_emerg_relay== 0 && relay_permission_2_4_20 == 0)
						{							
							xSemaphoreGive( Semaphore_Relay_2 );							
						}				
				}
				
		} //Закрываем условие на "прогрев"
		
		
		//Тест реле
		if (test_relay == 1 && menu_edit_mode == 0)
		{
			xSemaphoreGive( Semaphore_Relay_1 );
			state_warning_relay = 1;
			xSemaphoreGive( Semaphore_Relay_2 );
			state_emerg_relay = 1;
			osDelay(20000);
			test_relay = 0;
		}
		
		//Квитирование
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0 || settings[96] == 1 || (menu_horizontal == 0 && button_center_pressed_in_short == 1)) 
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			state_warning_relay = 0;
			
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
			state_emerg_relay = 0;
			
			trigger_event_attribute = 0;
			trigger_485_event_attribute_warning = 0;
			trigger_485_event_attribute_emerg = 0;
			
			settings[96] = 0;
			
			if (menu_horizontal == 0) //Если квитировали с помощью кнопки
			{
				button_center_pressed_in_short = 0;
				menu_edit_mode = 0;
			}
			
			quit_relay_button = 1;		

			//Сброс битов в системе ЗСК
			trigger_485_ZSK = 0;
			trigger_485_ZSK_percent = 0;
			x_axis = 0;
			y_axis = 0;
			z_axis = 0;			
			settings[30] = 0;
			settings[31] = 0;			
			
			//Чтоб пересчитывались проценты снова, при сбросе
			for(int i = 0; i < ZSK_REG_485_QTY; i++) 
			{				
				ZSK_trigger_array_previous[i] = 0;
			}
			
		}
		
		//Контроль напряжения питания ПЛК (+24 )
		if (power_supply_voltage < power_supply_warning_lo || power_supply_voltage > power_supply_warning_hi)
		{
			state_warning_relay = 1;							
			xSemaphoreGive( Semaphore_Relay_1 );	
		}
		
		
    osDelay(10);
  }
  /* USER CODE END TriggerLogic_Task */
}

/* USER CODE BEGIN Header_Relay_1_Task */
/**
* @brief Function implementing the myTask19 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Relay_1_Task */
void Relay_1_Task(void const * argument)
{
  /* USER CODE BEGIN Relay_1_Task */
	uint8_t prev_state_relay;
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Relay_1, portMAX_DELAY );		
		
		
		if (warming_flag == 0 && state_warning_relay == 1)
		{			
			if (state_warning_relay == 1)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);	//Замкнуто			
			}
				
			if (prev_state_relay == 0) warning_relay_counter++;
		}
		
		
		if (state_warning_relay == 0 && mode_relay == 0)
		{
			if (flag_for_delay_relay_exit == 1) 
			{ 
					osDelay(delay_relay_exit); 
					flag_for_delay_relay_exit = 0; 
			}			
			
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //Разомкнуто
		}
		
		prev_state_relay = state_warning_relay;
  }
  /* USER CODE END Relay_1_Task */
}

/* USER CODE BEGIN Header_Relay_2_Task */
/**
* @brief Function implementing the myTask20 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Relay_2_Task */
void Relay_2_Task(void const * argument)
{
  /* USER CODE BEGIN Relay_2_Task */
	uint8_t prev_state_relay;
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Relay_2, portMAX_DELAY );		
				
		
		if (warming_flag == 0 && state_emerg_relay == 1)
		{			
			
			
			if (state_emerg_relay == 1)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); //Разомкнуто		
			}

			if (prev_state_relay == 0) 
			{				
				emerg_relay_counter++;			 
			}
		}
		
		if (state_emerg_relay == 0 && mode_relay == 0)
		{
			if (flag_for_delay_relay_exit == 1) 
			{ 
					osDelay(delay_relay_exit); 
					flag_for_delay_relay_exit = 0; 
			}
			
			
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); //Замкнуто
		}   
		
		
		prev_state_relay = state_emerg_relay;
  }
  /* USER CODE END Relay_2_Task */
}

/* USER CODE BEGIN Header_HART_Receive_Task */
/**
* @brief Function implementing the myTask21 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HART_Receive_Task */
void HART_Receive_Task(void const * argument)
{
  /* USER CODE BEGIN HART_Receive_Task */
	uint16_t crc = 0;
	uint16_t count_registers = 0;
	uint16_t adr_of_registers = 0;
	uint16_t recieve_calculated_crc = 0;
	uint16_t recieve_actual_crc = 0;
	uint16_t outer_register = 0;
	uint8_t func_number = 0;
	uint16_t byte_qty = 0;
	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_HART_Receive, portMAX_DELAY );		
		
		__HAL_UART_CLEAR_IT(&huart5, UART_CLEAR_IDLEF); 				
		__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
		
		HAL_UART_DMAStop(&huart5); 				

		HAL_UART_Receive_DMA(&huart5, HART_receiveBuffer, 16);							
		
		
		if (HART_receiveBuffer[0] == hart_slave_address)
		{		
				
				func_number = HART_receiveBuffer[1]; //номер функции	
				byte_qty = HART_receiveBuffer[2];//кол-во байт
			
			  recieve_calculated_crc = crc16(HART_receiveBuffer, 6);
				recieve_actual_crc = (HART_receiveBuffer[7] << 8) + HART_receiveBuffer[6];
				
				//Проверяем crc
				if (recieve_calculated_crc == recieve_actual_crc) 
				{						
					if (HART_receiveBuffer[1] == 0x03 || HART_receiveBuffer[1] == 0x04) //Holding Register (FC=03)
					{
						hart_value = ( HART_receiveBuffer[3] << 8 ) + HART_receiveBuffer[4];
					}
						
				}
		}

    
  }
  /* USER CODE END HART_Receive_Task */
}

/* USER CODE BEGIN Header_HART_Transmit_Task */
/**
* @brief Function implementing the myTask22 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HART_Transmit_Task */
void HART_Transmit_Task(void const * argument)
{
  /* USER CODE BEGIN HART_Transmit_Task */
  /* Infinite loop */
	uint16_t crc = 0;
	
  /* Infinite loop */
  for(;;)
  {   
		
		if (hart_switch_on == 1)
		{

			HART_transmitBuffer[0] = hart_slave_address;
			HART_transmitBuffer[1] = hart_func;
			HART_transmitBuffer[2] = hart_slave_numreg >> 8;
			HART_transmitBuffer[3] = hart_slave_numreg & 0x00FF;
			HART_transmitBuffer[4] = hart_regs_qty >> 8;
			HART_transmitBuffer[5] = hart_regs_qty & 0x00FF;		
			
			crc = crc16(HART_transmitBuffer, 6);				
						
			HART_transmitBuffer[6] = crc;
			HART_transmitBuffer[7] = crc >> 8;	
			
			HAL_UART_Transmit_DMA(&huart5, HART_transmitBuffer, 8);
			
		}
		
		osDelay(1000);
		
  }
  /* USER CODE END HART_Transmit_Task */
}

/* USER CODE BEGIN Header_TBUS_Modbus_Receive_Task */
/**
* @brief Function implementing the myTask23 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TBUS_Modbus_Receive_Task */
void TBUS_Modbus_Receive_Task(void const * argument)
{
  /* USER CODE BEGIN TBUS_Modbus_Receive_Task */
	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_TBUS_Modbus_Rx, portMAX_DELAY );					
						
		__HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_IDLEF); 				
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
		
		HAL_UART_DMAStop(&huart3); 
				
		HAL_UART_Receive_DMA(&huart3, TBUS_receiveBuffer, 256);					
			
		xSemaphoreGive( Semaphore_TBUS_Modbus_Tx );
						
   
  }
  /* USER CODE END TBUS_Modbus_Receive_Task */
}

/* USER CODE BEGIN Header_TBUS_Modbus_Transmit_Task */
/**
* @brief Function implementing the myTask24 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TBUS_Modbus_Transmit_Task */
void TBUS_Modbus_Transmit_Task(void const * argument)
{
  /* USER CODE BEGIN TBUS_Modbus_Transmit_Task */
	uint16_t crc = 0;
	volatile uint16_t count_registers = 0;
	volatile uint16_t adr_of_registers = 0;
	volatile uint16_t recieve_calculated_crc = 0;
	volatile uint16_t recieve_actual_crc = 0;
	volatile uint16_t outer_register = 0;
	volatile uint16_t number_of_bytes_further = 0;
	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_TBUS_Modbus_Tx, portMAX_DELAY );
		
		//for (int i = 0; i < REG_COUNT*2+5; i++) transmitBuffer[i] = 0;
		
		if (TBUS_receiveBuffer[0] == SLAVE_ADR)
		{		
				recieve_calculated_crc = crc16(TBUS_receiveBuffer, 6);
				recieve_actual_crc = (TBUS_receiveBuffer[7] << 8) + TBUS_receiveBuffer[6];
				
				//Если 16 функция, другая длина пакета
				if (TBUS_receiveBuffer[1] == 0x10) 
				{
					number_of_bytes_further = TBUS_receiveBuffer[6];
					recieve_calculated_crc = crc16(TBUS_receiveBuffer, 7 + number_of_bytes_further);
					recieve_actual_crc = (TBUS_receiveBuffer[7 + number_of_bytes_further + 1] << 8) + TBUS_receiveBuffer[7 + number_of_bytes_further];
				}
				
				//Проверяем crc
				if (recieve_calculated_crc == recieve_actual_crc) 
				{	
						TBUS_transmitBuffer[0] = TBUS_receiveBuffer[0]; //адрес устр-ва			
						TBUS_transmitBuffer[1] = TBUS_receiveBuffer[1]; //номер функции						
					
						adr_of_registers = (TBUS_receiveBuffer[2] << 8) + TBUS_receiveBuffer[3];//получаем адрес регистра				
						count_registers = (TBUS_receiveBuffer[4] << 8) + TBUS_receiveBuffer[5]; //получаем кол-во регистров из запроса
						outer_register = adr_of_registers + count_registers; //крайний регистр
						
						TBUS_transmitBuffer[2] = count_registers*2; //количество байт	(в два раза больше чем регистров)	
					
					
//						//Проверяем номер регистра
//						if (adr_of_registers > REG_COUNT) 
//						{
//									if (transmitBuffer[1] == 0x3) transmitBuffer[1] = 0x83; //Function Code in Exception Response
//									if (transmitBuffer[1] == 0x4) transmitBuffer[1] = 0x84; //Function Code in Exception Response
//									
//									transmitBuffer[2] = 0x02; //Exception "Illegal Data Address"		
//									
//									crc = crc16(transmitBuffer, 3);
//							
//									transmitBuffer[3] = crc;
//									transmitBuffer[4] = crc >> 8;		 
//							
//									HAL_UART_Transmit_DMA(&huart2, transmitBuffer, 5);									
//						}					
						
						if (TBUS_receiveBuffer[1] == 0x03 || TBUS_receiveBuffer[1] == 0x04) //Holding Register (FC=03) or Input Register (FC=04)
						{		
									if (adr_of_registers < 125) //если кол-во регистров больше 125 (255 байт макс.), опрос идет несколькими запросами 
									{							
											for (volatile uint16_t i=adr_of_registers, j=0; i < outer_register; i++, j++)
											{
												TBUS_transmitBuffer[j*2+3] = settings[i] >> 8; //значение регистра Lo 		
												TBUS_transmitBuffer[j*2+4] = settings[i] & 0x00FF; //значение регистра Hi		
											}
									
											crc = crc16(TBUS_transmitBuffer, count_registers*2+3);				
									
											TBUS_transmitBuffer[count_registers*2+3] = crc;
											TBUS_transmitBuffer[count_registers*2+3+1] = crc >> 8;		
																				
												
											HAL_UART_Transmit_DMA(&huart3, TBUS_transmitBuffer, count_registers*2+5);
									}
									else
									{
											if (adr_of_registers < 944)
											{
													for (uint16_t i=0, j=0; i < count_registers; i++, j++)
													{
														TBUS_transmitBuffer[j*2+3] = settings[adr_of_registers + i] >> 8; //значение регистра Lo 		
														TBUS_transmitBuffer[j*2+4] = settings[adr_of_registers + i] & 0x00FF; //значение регистра Hi		
													}
											}
											
											if (adr_of_registers > 944)
											{
													for (uint16_t i=0, j=0; i < MIRROR_COUNT; i++, j++)
													{
														TBUS_transmitBuffer[j*2+3] = mirror_values[i] >> 8; //значение регистра Lo 		
														TBUS_transmitBuffer[j*2+4] = mirror_values[i] & 0x00FF; //значение регистра Hi		
													}
											}
											
											if (adr_of_registers > 1080)
											{
													for (uint16_t i=0, j=0; i < MIRROR_COUNT; i++, j++)
													{
														TBUS_transmitBuffer[j*2+3] = 0; //значение регистра Lo 		
														TBUS_transmitBuffer[j*2+4] = 0; //значение регистра Hi		
													}
											}
									
											crc = crc16(TBUS_transmitBuffer, count_registers*2+3);				
									
											TBUS_transmitBuffer[count_registers*2+3] = crc;
											TBUS_transmitBuffer[count_registers*2+3+1] = crc >> 8;		
																				
															
											HAL_UART_Transmit_DMA(&huart3, TBUS_transmitBuffer, count_registers*2+5);					
									}
						}							
						else if (TBUS_receiveBuffer[1] == 0x06 && menu_edit_mode == 0) //Preset Single Register (FC=06)
						{									
									if (adr_of_registers == 107)
									{
										settings[adr_of_registers] = (TBUS_receiveBuffer[4] << 8) + TBUS_receiveBuffer[5]; 										
									}
									else if (settings[107] == -7035) //Если изменяем регистры, то надо снять блокировку (Рег.108 = 0xE485)
									{
										settings[adr_of_registers] = (TBUS_receiveBuffer[4] << 8) + TBUS_receiveBuffer[5]; 										
									}
										

									TBUS_transmitBuffer[2] = TBUS_receiveBuffer[2];
									TBUS_transmitBuffer[3] = TBUS_receiveBuffer[3];
							
									TBUS_transmitBuffer[4] = TBUS_receiveBuffer[4];
									TBUS_transmitBuffer[5] = TBUS_receiveBuffer[5];
							
									crc = crc16(TBUS_transmitBuffer, 6);				
							
									TBUS_transmitBuffer[6] = crc;
									TBUS_transmitBuffer[7] = crc >> 8;		
																		
							
									HAL_UART_Transmit_DMA(&huart3, TBUS_transmitBuffer, 8);						
						}				
						else if (TBUS_receiveBuffer[1] == 0x10 && menu_edit_mode == 0) //Preset Multiply Registers (FC=16)
						{									
							
									if (adr_of_registers == 107)
									{
										settings[adr_of_registers] = (TBUS_receiveBuffer[7] << 8) + TBUS_receiveBuffer[8]; 										
										settings[adr_of_registers+1] = (TBUS_receiveBuffer[9] << 8) + TBUS_receiveBuffer[10];
									}
									else if (settings[107] == -7035) //Если изменяем регистры, то надо снять блокировку (Рег.108 = 0xE485)
									{
										for (uint16_t i=adr_of_registers, j=0; i < outer_register; i++, j=j+2)
										{											
											settings[i] = (TBUS_receiveBuffer[7 + j] << 8) + TBUS_receiveBuffer[8 + j]; 										
											//settings[adr_of_registers+1] = (TBUS_receiveBuffer[9] << 8) + TBUS_receiveBuffer[10];
										}
									}										
									

									TBUS_transmitBuffer[2] = TBUS_receiveBuffer[2];//адрес первого регистра
									TBUS_transmitBuffer[3] = TBUS_receiveBuffer[3];
							
									TBUS_transmitBuffer[4] = TBUS_receiveBuffer[4];//кол-во регистров	
									TBUS_transmitBuffer[5] = TBUS_receiveBuffer[5];
								
							
									crc = crc16(TBUS_transmitBuffer, 6);				
							
									TBUS_transmitBuffer[6] = crc;
									TBUS_transmitBuffer[7] = crc >> 8;		
									
							
									HAL_UART_Transmit_DMA(&huart3, TBUS_transmitBuffer, 8);						
						}
						else
						{							
									TBUS_transmitBuffer[1] = 0x81; //Function Code in Exception Response
									TBUS_transmitBuffer[2] = 0x01; //Exception "Illegal function"			
									
									crc = crc16(TBUS_transmitBuffer, 3);
							
									TBUS_transmitBuffer[3] = crc;
									TBUS_transmitBuffer[4] = crc >> 8;		 
							
									HAL_UART_Transmit_DMA(&huart3, TBUS_transmitBuffer, 5);
						}						
				}	
		}   
  }
  /* USER CODE END TBUS_Modbus_Transmit_Task */
}

/* USER CODE BEGIN Header_Update_Bootloader_Task */
/**
* @brief Function implementing the myTask25 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Update_Bootloader_Task */
void Update_Bootloader_Task(void const * argument)
{
  /* USER CODE BEGIN Update_Bootloader_Task */
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Bootloader_Update, portMAX_DELAY );					

	  data = ((uint64_t) boot_receiveBuffer[0]) + 
					((uint64_t) (boot_receiveBuffer[1]) << 8) + 
					((uint64_t) (boot_receiveBuffer[2]) << 16) + 
					((uint64_t) (boot_receiveBuffer[3]) << 24) + 
					((uint64_t) (boot_receiveBuffer[4]) << 32) + 
					((uint64_t) (boot_receiveBuffer[5]) << 40) + 
					((uint64_t) (boot_receiveBuffer[6]) << 48) + 
					((uint64_t) (boot_receiveBuffer[7]) << 56);
		
		//Programm
		if (worker_status == 0x04)
		{			
				//__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);	
			
				status = HAL_FLASH_Unlock();							
				
				status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (BOOT_START_ADDRESS) + 8*byte_bunch, data);						
				
				HAL_FLASH_Lock();	
				
				byte_bunch++; byte_counter += 8;
			
				if (byte_counter >= byte_size) 
				{
					
					crc_flash = flash_crc16(BOOT_START_ADDRESS, byte_size);
					
					if (crc_flash == crc_data)
					{
						
						FLASH_EraseInitTypeDef EraseInitStruct;							
						uint32_t PAGEError = 0;
						EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
						EraseInitStruct.Banks = 1;
						EraseInitStruct.Page = 6;
						EraseInitStruct.NbPages = 1;					

						__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);					
						status = HAL_FLASH_Unlock();	
							
						status1 = HAL_FLASHEx_Erase(&EraseInitStruct,&PAGEError);															
						osDelay(3);

						__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);											
						status2 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, BOOT_CRC_ADR, crc_data);																	
						osDelay(3);
						
						
						__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);						
						status3 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, BOOT_SIZE, byte_size);						
						osDelay(3);					
						
						
						HAL_FLASH_Lock();
						
						if (status1 == 0 && status2 == 0 && status3 == 0) worker_status = 5;
						osDelay(3000);	
					}
					
					JumpToApplication( BOOT_START_ADDRESS );		
				}
											
		}

		//Get crc
		if (worker_status == 0x03)
		{
				
				crc_data = data;				
				worker_status = 0x04;
				byte_counter = 0;
				byte_bunch = 0;
				error_crc = 0;	
		}
		
		//Get size
		if (worker_status == 0x02)
		{				
				byte_size = data >> 32;				
				worker_status = 0x03;
				byte_counter = 0;
				byte_bunch = 0;
				error_crc = 0;	
		}		
		
		if (worker_status == 0x01)
		{				
				worker_status = 0x02;			
		}
		
	
		//Зажечь светодиод
		HAL_GPIO_WritePin(GPIOC, SD2_Pin, GPIO_PIN_SET);
		osDelay(1);
		HAL_GPIO_WritePin(GPIOC, SD2_Pin, GPIO_PIN_RESET);		
  	
  }
  /* USER CODE END Update_Bootloader_Task */
}

/* USER CODE BEGIN Header_Erase_Bootloader_Task */
/**
* @brief Function implementing the myTask26 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Erase_Bootloader_Task */
void Erase_Bootloader_Task(void const * argument)
{
  /* USER CODE BEGIN Erase_Bootloader_Task */
  /* Infinite loop */
  for(;;)
  {		
			xSemaphoreTake( Semaphore_Bootloader_Erase, portMAX_DELAY );					
		
			if (bootloader_state == 0x01)
			{
				FLASH_EraseInitTypeDef EraseInitStruct;					
				uint32_t PAGEError = 0;
				EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
				EraseInitStruct.Banks = 1;
				EraseInitStruct.Page = 8;
				EraseInitStruct.NbPages = 24;
			
				__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
				
				status = HAL_FLASH_Unlock();	
				osDelay(3);				
							
				status = HAL_FLASHEx_Erase(&EraseInitStruct,&PAGEError);				
				
				//status = HAL_FLASH_GetError();
				
				HAL_FLASH_Lock();	
			
				if (status == HAL_OK) worker_status = 0x01;
			}
  }
  /* USER CODE END Erase_Bootloader_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


void Integrate_V(float32_t* input, float32_t* output, uint32_t size)
{	
	
	for (uint16_t i=0; i < size; i++)
	{							
		output[i] = input[i] / 6.4f + integrator_summa_V;		

		integrator_summa_V = output[i]; 			
	}		

}

void Integrate_D(float32_t* input, float32_t* output, uint32_t size)
{	
	
	for (uint16_t i=0; i < size; i++)
	{							
		output[i] = input[i] / 6.4f + integrator_summa_D;		

		integrator_summa_D = output[i]; 		

	}	
	
}


void FilterInit(void)
{                       

//			//Баттерворт 3п 1100 Гц (25600)	
//			static float32_t coef_main_low_gain_25600[] = {				
//				1*0.013361128677806023,  2*0.013361128677806023,  1*0.013361128677806023,  1.729897146458744,  -0.78334166116996795,        
//				1*0.10979617017302817,  1*0.10979617017302817,  0*0.10979617017302817,  0.78040765965394354,  0                          
//			};	
	
//				//Баттерворт 3п 1200 Гц (25600)	
//				static float32_t coef_main_low_gain_25600[] = {					
//					1*0.018801009627942851,  2*0.018801009627942851,  1*0.018801009627942851,  1.6713037383271774, -0.74650777683894864,        
//					1*0.12917472686398232,  1*0.12917472686398232,  0*0.12917472686398232,  0.74165054627203542,  0                          
//				};    				
				

				//Баттерворт 3п 1300 Гц (25600)	
				static float32_t coef_main_low_gain_25600[] = {					
					1*0.021814503924926908,  2*0.021814503924926908,  1*0.021814503924926908,  1.6415882340487031,  -0.72884624974841106,        
					1*0.13860037351789706,  1*0.13860037351789706,  0*0.13860037351789706,  0.72279925296420588,  0                                                    
				};                                                                                                   
  	

//				//Баттерворт 3п 2000 Гц (25600)	
//				static float32_t coef_main_low_gain_25600[] = {					
//					1*0.047778138528779025,  2*0.047778138528779025,  1*0.047778138528779025,  1.4274054039271769,  -0.61851795804229315,        
//					1*0.2003115331590381,  	 1*0.2003115331590381,    0*0.2003115331590381,    0.59937693368192368,  0                            	
//				};		

//			//Баттерворт 3п 1000Гц (6400)
//			static float32_t coef_main_low_gain[] = {					
//				1*0.18342043459415436,  2*0.18342043459415436,  1*0.18342043459415436,  0.65428119897842407, -0.38796296715736389,       
//				1*0.37475651502609253,  1*0.37475651502609253,  0*0.37475651502609253,  0.25048696994781494,  0                          
//			};

//			//Баттерворт 3п 1100Гц (6400)
//			static float32_t coef_main_low_gain[] = {								
//				1*0.18342043889721857,  2*0.18342043889721857,  1*0.18342043889721857,  0.65428121532333072, -0.38796297091220483,        
//				1*0.37475651990434722,  1*0.37475651990434722,  0*0.37475651990434722,  0.25048696019130551,  0                          
//			};

				//Баттерворт 3п 1100Гц (6400)
				static float32_t coef_main_low_gain[] = {								
					1*0.21112927559799433,  2*0.21112927559799433,  1*0.21112927559799433,  0.52352831655332599, -0.36804541894530324,        
					1*0.400543816310171,  1*0.400543816310171,  0*0.400543816310171,  0.19891236737965803,  0                          
				};
                                                             

                                          
  
                                         
			//Баттерворт, 3п, 1.9Гц (6400)
			static float32_t coef_main_highpass_2Hz_gain[] = {			
				1*0.99906734022106236,  -2*0.99906734022106236,  1*0.99906734022106236,  1.9981329423531335, -0.99813641853111601,      
				1*0.99906820845578981,  -1*0.99906820845578981,  0*0.99906820845578981,  0.99813641691157962,  0                          
			}; 
                                               
			//Баттерворт, 3п, 3.7Гц (6400)
			static float32_t coef_main_highpass_5Hz_gain[] = {                                                 		                                                           
				1*0.99818377073042208,  -2*0.99818377073042208,  1*0.99818377073042208,  1.996360956022307, -0.99637412689938132,       
				1*0.99818705748017988,  -1*0.99818705748017988,  0*0.99818705748017988, 0.99637411496035977,  0                         
			};
			
			//Баттерворт, 3п, 7.5Гц (6400)
			static float32_t coef_main_highpass_10Hz_gain[] = {		             						
				1*0.99631847919207916,  -2*0.99631847919207916,  1*0.99631847919207916,  1.9926099502594936, -0.99266396650882305,        
				1*0.9963319337206159,  -1*0.9963319337206159,  0*0.9963319337206159,  0.9926638674412317,  0                            
			};


                          
		

		if (FILTER_MODE <= 0) 		
		{
			arm_biquad_cascade_df1_init_f32(&filter_main_high_icp, 2, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_main_high_icp[0]);				
						
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_icp, 2, (float32_t *) &coef_main_highpass_2Hz_gain[0], &pStates_highpass_1_icp[0]);							
										
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_icp, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_2_icp[0]);				
			
			
			arm_biquad_cascade_df1_init_f32(&filter_main_low_icp, 2, (float32_t *) &coef_main_low_gain_25600[0], &pStates_main_low_icp[0]);				
			arm_biquad_cascade_df1_init_f32(&filter_main_low_icp_2, 2, (float32_t *) &coef_main_low_gain[0], &pStates_main_low_icp_2[0]);				
			
		}
		else if (FILTER_MODE == 1)		
		{
			arm_biquad_cascade_df1_init_f32(&filter_main_high_icp, 2, (float32_t *) &coef_main_highpass_5Hz_gain[0], &pStates_main_high_icp[0]);				
						
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_icp, 2, (float32_t *) &coef_main_highpass_5Hz_gain[0], &pStates_highpass_1_icp[0]);							
							
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_icp, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_2_icp[0]);							
			
			
			arm_biquad_cascade_df1_init_f32(&filter_main_low_icp, 2, (float32_t *) &coef_main_low_gain_25600[0], &pStates_main_low_icp[0]);	
			arm_biquad_cascade_df1_init_f32(&filter_main_low_icp_2, 2, (float32_t *) &coef_main_low_gain[0], &pStates_main_low_icp_2[0]);				
		}
		else if (FILTER_MODE >= 2)		
		{
			arm_biquad_cascade_df1_init_f32(&filter_main_high_icp, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_main_high_icp[0]);				
							
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_1_icp, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_1_icp[0]);							
							
			arm_biquad_cascade_df1_init_f32(&filter_instance_highpass_2_icp, 2, (float32_t *) &coef_main_highpass_10Hz_gain[0], &pStates_highpass_2_icp[0]);				
			
			
			arm_biquad_cascade_df1_init_f32(&filter_main_low_icp, 2, (float32_t *) &coef_main_low_gain_25600[0], &pStates_main_low_icp[0]);										
			arm_biquad_cascade_df1_init_f32(&filter_main_low_icp_2, 2, (float32_t *) &coef_main_low_gain[0], &pStates_main_low_icp_2[0]);				
		}	
		
		integrator_summa_V = 0.0;
		integrator_summa_D = 0.0;
	
}


uint32_t rtc_read_backup_reg(uint32_t BackupRegister) 
{
    RTC_HandleTypeDef RtcHandle;
    RtcHandle.Instance = RTC;
    return HAL_RTCEx_BKUPRead(&RtcHandle, BackupRegister);
}
 
void rtc_write_backup_reg(uint32_t BackupRegister, uint32_t data) 
{
    RTC_HandleTypeDef RtcHandle;
    RtcHandle.Instance = RTC;
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&RtcHandle, BackupRegister, data);
    HAL_PWR_DisableBkUpAccess();
}

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
	
	if (channel_485_ON == 1 || channel_485_ON == 2)
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
	
	
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 0 && where_from == 1) 
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
	
			menu_edit_mode = 0;
	
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

void JumpToApplication(uint32_t ADDRESS)
{
		typedef  void (*pFunction)(void);
		uint32_t  JumpAddress = *(__IO uint32_t*)(ADDRESS + 4);
    pFunction Jump = (pFunction)JumpAddress;
        
    HAL_DeInit();
    
    __set_CONTROL(0); 
    __set_MSP(*(__IO uint32_t*) ADDRESS);
		SCB->VTOR = ADDRESS;
		Jump();		 
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
