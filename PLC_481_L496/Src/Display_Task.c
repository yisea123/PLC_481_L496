#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "stm32l4xx_hal.h"
#include "main.h"
#include "arm_math.h"
#include "math.h"
#include "Display_Task.h"
#include "ssd1306.h"
#include <stdint.h>
#include "fonts.h"

char msg[30];
uint16_t temp_buf[2];

extern uint8_t warming_flag;
extern uint8_t menu_edit_mode; //Режим редактирования
extern uint16_t menu_index_pointer;
extern int16_t icp_menu_points_for_showing;
extern int16_t menu_485_points_for_showing;
extern uint8_t menu_edit_settings_mode;
extern uint16_t menu_index;
extern uint16_t menu_index_array[];
extern uint16_t menu_index_pointer;
extern uint16_t menu_vertical;
extern uint16_t menu_horizontal;
extern uint8_t horizont_menu_lenght;
extern uint8_t vertical_menu_lenght;
extern uint8_t button_left_pressed_in;
extern uint8_t button_right_pressed_in;
extern uint8_t button_up_pressed_in;
extern uint8_t button_down_pressed_in;
extern uint8_t button_center_pressed_in_short;
extern uint8_t button_center_pressed_in_long;
extern uint8_t temp_str; //Скроллинг (промотка) строки в меню
extern uint8_t menu_edit_mode; //Режим редактирования
extern uint8_t digit_rank; //Разряд числа (для редактирования)
extern float32_t fractpart;
extern uint16_t number_of_items_in_the_menu;
extern uint8_t quit_relay_button;
extern uint16_t channel_ICP_ON;
extern uint16_t channel_4_20_ON;
extern uint16_t channel_485_ON;
extern FontDef font_8x15_RU;
extern FontDef font_8x14;
extern uint8_t break_sensor_icp;
extern uint8_t break_sensor_485;
extern uint8_t break_sensor_420;
extern float32_t rms_acceleration_icp;
extern float32_t rms_velocity_icp;
extern float32_t rms_displacement_icp;
extern char buffer[];
extern uint8_t temp_stat_1;
extern float32_t max_acceleration_icp;
extern float32_t min_acceleration_icp;
extern float32_t max_velocity_icp;
extern float32_t min_velocity_icp;
extern float32_t max_displacement_icp;
extern float32_t min_displacement_icp;
extern uint8_t disable_up_down_button;
extern uint8_t filter_mode_icp;

void Display_Task_Init(void)
{	
	// CS# (This pin is the chip select input. (active LOW))
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	
	ssd1306_Init();
	ssd1306_Fill(1);
	check_logo();
	ssd1306_UpdateScreen();
	//osDelay(settings[109]/2); //Заливка половина времени прогрева
	osDelay(1000);

	init_menu(1);
}

void Display_Task_Core(void)
{
			if (warming_flag == 1) 
			{				
				ssd1306_Fill(0);				
				
				logo();
				
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
						if (menu_edit_settings_mode == 0) horizont_menu_lenght = 3;
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
						quit_relay_button = 1; //Включаем таймер чтоб не срабатывало квитирование						
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
					if (channel_485_ON == 1)
					{
						
							

							if (menu_index_pointer == 3 && menu_horizontal == 0) //Значение регистра
							{
								ssd1306_Fill(0);
								ssd1306_SetCursor(0,0);												
								ssd1306_WriteString("485",font_8x14,1);		
								
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
										
										//if (menu_485_points_for_showing != 0)	
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
						
						ssd1306_SetCursor(0,15);						
						strncpy(msg,"Аттрибут события", 16);						
						string_scroll(msg, 16);
						ssd1306_WriteString(" 1",font_8x14,1);
						
						ssd1306_SetCursor(0,32);														
						snprintf(buffer, sizeof buffer, "0x%X", trigger_event_attribute);			
						ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим

						menu_edit_mode = 0 ; //Запрещаем редактирование						
						
						ssd1306_UpdateScreen();			

						disable_up_down_button = 1;
					}						
					
					if (menu_index_pointer == 4 && menu_horizontal == 2 && menu_edit_settings_mode == 0) //Аттрибут события 485 пред. уставка
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			

						triangle_left(55,0);
						triangle_right(60,0);																								
						//triangle_up(58,38);
						//triangle_down(58,43);
						
						ssd1306_SetCursor(0,15);	
						
						strncpy(msg,"Аттрибут события", 16);						
						string_scroll(msg, 16);
						ssd1306_WriteString(" 2",font_8x14,1);
						
						ssd1306_SetCursor(0,32);														
						snprintf(buffer, sizeof buffer, "0x%X", trigger_485_event_attribute_warning);			
						ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим

						menu_edit_mode = 0 ; //Запрещаем редактирование
						
						//ssd1306_UpdateScreen();		

						disable_up_down_button = 1;
					}						
					
					if (menu_index_pointer == 4 && menu_horizontal == 3 && menu_edit_settings_mode == 0) //Аттрибут события 485 авар. уставка
					{
						ssd1306_Fill(0);
						ssd1306_SetCursor(0,0);												
						ssd1306_WriteString("Реле",font_8x15_RU,1);			
						
						triangle_left(55,0);																														
						//triangle_up(58,38);
						//triangle_down(58,43);
						
						ssd1306_SetCursor(0,15);						
						strncpy(msg,"Аттрибут события", 16);						
						string_scroll(msg, 16);
						ssd1306_WriteString(" 3",font_8x14,1);
						
						ssd1306_SetCursor(0,32);														
						snprintf(buffer, sizeof buffer, "0x%X", trigger_485_event_attribute_emerg);			
						ssd1306_WriteString(buffer,font_8x14,1); //Рабочий режим

						menu_edit_mode = 0 ; //Запрещаем редактирование
						
						//ssd1306_UpdateScreen();				
						
						disable_up_down_button = 1;
					}							
					
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
}
 