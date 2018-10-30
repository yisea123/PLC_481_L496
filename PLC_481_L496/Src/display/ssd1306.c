
#include "ssd1306.h"
#include "spi.h"
#include <string.h>

extern uint8_t tik_logo[];
extern uint8_t tik_logo2[];
extern uint8_t tik_logo_bitmap2[];


static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
uint16_t size_buffer = SSD1306_WIDTH * SSD1306_HEIGHT / 8; 

SSD1306_t SSD1306;


static void ssd1306_WriteCommand(uint8_t command)
{
	uint8_t* receive;
	
	// D/C# (This is Data/Command control pin. HIGH - data, LOW - command)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);	
	// CS# (This pin is the chip select input. (active LOW))
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
		
	
	HAL_SPI_Transmit(&hspi3, (uint8_t*) &command, sizeof(command), 1000);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // D/C#
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // CS#	
}


//
//	Het scherm initialiseren voor gebruik
//
uint8_t ssd1306_Init(void)
{	

	
	// RES#
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);	
	HAL_Delay(10);	
	
	
	/* Init LCD */
//	ssd1306_WriteCommand(0xAE); //display off
//	ssd1306_WriteCommand(0x20); //Set Memory Addressing Mode   
//	ssd1306_WriteCommand(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
//	ssd1306_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
//	ssd1306_WriteCommand(0xC8); //Set COM Output Scan Direction
//	ssd1306_WriteCommand(0x00); //---set low column address
//	ssd1306_WriteCommand(0x10); //---set high column address
//	ssd1306_WriteCommand(0x40); //--set start line address
//	ssd1306_WriteCommand(0x81); //--set contrast control register
//	ssd1306_WriteCommand(0xFF);
//	ssd1306_WriteCommand(0xA1); //--set segment re-map 0 to 127
//	ssd1306_WriteCommand(0xA6); //--set normal display
//	ssd1306_WriteCommand(0xA8); //--set multiplex ratio(1 to 64)
//	ssd1306_WriteCommand(0x3F); //
//	ssd1306_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
//	ssd1306_WriteCommand(0xD3); //-set display offset
//	ssd1306_WriteCommand(0x00); //-not offset
//	ssd1306_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
//	ssd1306_WriteCommand(0xF0); //--set divide ratio
//	ssd1306_WriteCommand(0xD9); //--set pre-charge period
//	ssd1306_WriteCommand(0x22); //
//	ssd1306_WriteCommand(0xDA); //--set com pins hardware configuration
//	ssd1306_WriteCommand(0x12);
//	ssd1306_WriteCommand(0xDB); //--set vcomh
//	ssd1306_WriteCommand(0x20); //0x20,0.77xVcc
//	ssd1306_WriteCommand(0x8D); //--set DC-DC enable
//	ssd1306_WriteCommand(0x14); //
//	ssd1306_WriteCommand(0xAF); //--turn on SSD1306 panel

ssd1306_WriteCommand(0xAE);
ssd1306_WriteCommand(0xA8);
ssd1306_WriteCommand(0x3F);
ssd1306_WriteCommand(0xD3);
ssd1306_WriteCommand(0x00);
ssd1306_WriteCommand(0x40);
ssd1306_WriteCommand(0xA0);
ssd1306_WriteCommand(0xC0);
ssd1306_WriteCommand(0xDA);
ssd1306_WriteCommand(0x12);
//ssd1306_WriteCommand(0x7F);
//ssd1306_WriteCommand(0xA4);
//ssd1306_WriteCommand(0xA0);
//ssd1306_WriteCommand(0xC0);
//ssd1306_WriteCommand(0x20);
//ssd1306_WriteCommand(0x00);
//ssd1306_WriteCommand(0xD5);
ssd1306_WriteCommand(0x80);
ssd1306_WriteCommand(0x8D);
ssd1306_WriteCommand(0x14);
//ssd1306_WriteCommand(0xDB);
//ssd1306_WriteCommand(0x20);
//ssd1306_WriteCommand(0xD9);
ssd1306_WriteCommand(255); //€ркость
ssd1306_WriteCommand(0x21);
ssd1306_WriteCommand(32);
ssd1306_WriteCommand(32+63);
ssd1306_WriteCommand(0x20);
ssd1306_WriteCommand(0);
ssd1306_WriteCommand(0); //позици€ старт
ssd1306_WriteCommand(0xAF);



	
	/* Clearen scherm */
	ssd1306_Fill(0);
	
	/* Update screen */
	ssd1306_UpdateScreen();
	
	/* Set default values */
	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;
	
	/* Initialized OK */
	SSD1306.Initialized = 1;
	
	/* Return OK */
	return 1;
}


void ssd1306_Fill(SSD1306_COLOR color) 
{
	/* Set memory */
	uint16_t i;
	
	
	for(int i = 0; i < sizeof(SSD1306_Buffer); i++)
	{
		SSD1306_Buffer[i] = (color == 0) ? 0x00 : 0xFF;
	}
}


void ssd1306_UpdateScreen(void) 
{
	uint8_t i;	
	
	// D/C# (This is Data/Command control pin. HIGH - data, LOW - command)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	
	// CS# (This pin is the chip select input. (active LOW))
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	
	for (i = 0; i < 8; i++) 
	{
		//ssd1306_WriteCommand(0xB0 + i);
		//ssd1306_WriteCommand(0x00);
		//ssd1306_WriteCommand(0x10);
		
		HAL_SPI_Transmit(&hspi3,&SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 1000);	
	}
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}


//	1 pixel op het scherm tekenen
//	X => X coordinaat
//	Y => Y coordinaat
//	color => kleur die pixel moet krijgen

void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) 
	{
		// We gaan niet buiten het scherm schrijven
		return;
	}
	
	// Kijken of de pixel geinverteerd moet worden
	if (SSD1306.Inverted) 
	{
		color = (SSD1306_COLOR)!color;
	}
	
	// We zetten de juiste kleur voor de pixel
	if (color == White)
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} 
	else 
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

//
//	We willen 1 char naar het scherm sturen
//	ch 		=> char om weg te schrijven
//	Font 	=> Font waarmee we gaan schrijven
//	color 	=> Black or White
//
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
	uint32_t i, b, j;
	
	// Kijken of er nog plaats is op deze lijn
	if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
	{
		// Er is geen plaats meer
		return 0;
	}
		
	
	// We gaan door het font
	for (i = 0; i < Font.FontHeight; i++)
	{
		if ((int)ch >= 192) 
		{
			b = Font.data[(ch - 191) * Font.FontHeight + i]; 			
		}
		else 
		{
			b = Font.data[(ch - 32) * Font.FontHeight + i];
		}
		
		for (j = 0; j < Font.FontWidth; j++)
		{
			if ((b << j) & 0x8000) 
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
			} 
			else 
			{
				//ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
			}
		}
	}
	
	// De huidige positie is nu verplaatst
	SSD1306.CurrentX += Font.FontWidth;
	
	// We geven het geschreven char terug voor validatie
	return ch;
}

//
//	Functie voor het wegschrijven van een hele string
// 	str => string om op het scherm te schrijven
//	Font => Het font dat gebruikt moet worden
//	color => Black or White
//
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color)
{
	// We schrijven alle char tot een nulbyte
	while (*str) 
	{
		if (ssd1306_WriteChar(*str, Font, color) != *str)
		{
			// Het karakter is niet juist weggeschreven
			return *str;
		}
		
		// Volgende char
		str++;
	}
	
	// Alles gelukt, we sturen dus 0 terug
	return *str;
}

//
//	Zet de cursor op een coordinaat
//
void ssd1306_SetCursor(uint8_t x, uint8_t y) 
{
	/* Set write pointers */
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

void triangle_down(uint8_t start_x, uint8_t start_y)
{
//		ssd1306_DrawPixel((uint8_t)start_x, (uint8_t)start_y, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x, (uint8_t)start_y+1, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y+1, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y+2, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+2, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+3, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x+3, (uint8_t)start_y+3, (SSD1306_COLOR)1);						
//		ssd1306_DrawPixel((uint8_t)start_x+3, (uint8_t)start_y+4, (SSD1306_COLOR)1);						
//		ssd1306_DrawPixel((uint8_t)start_x+4, (uint8_t)start_y+4, (SSD1306_COLOR)1);						
//		ssd1306_DrawPixel((uint8_t)start_x+4, (uint8_t)start_y+5, (SSD1306_COLOR)1);						
//	
//		ssd1306_DrawPixel((uint8_t)start_x+4, (uint8_t)start_y+4, (SSD1306_COLOR)1);						
//		ssd1306_DrawPixel((uint8_t)start_x+5, (uint8_t)start_y+3, (SSD1306_COLOR)1);						
//		ssd1306_DrawPixel((uint8_t)start_x+6, (uint8_t)start_y+2, (SSD1306_COLOR)1);						
//		ssd1306_DrawPixel((uint8_t)start_x+7, (uint8_t)start_y+1, (SSD1306_COLOR)1);						
//		ssd1306_DrawPixel((uint8_t)start_x+8, (uint8_t)start_y, (SSD1306_COLOR)1);						
	
//		ssd1306_DrawPixel((uint8_t)start_x, (uint8_t)start_y, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y, (SSD1306_COLOR)1);
////		ssd1306_DrawPixel(start_x+2, start_y, 1);
////		ssd1306_DrawPixel(start_x+3, start_y, 1);
////		ssd1306_DrawPixel(start_x+4, start_y, 1);
//		ssd1306_DrawPixel((uint8_t)start_x+5, (uint8_t)start_y, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x+6, (uint8_t)start_y, (SSD1306_COLOR)1);						
		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y+1, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+1, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel(start_x+3, start_y+1, 1);
		ssd1306_DrawPixel((uint8_t)start_x+4, (uint8_t)start_y+1, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+5, (uint8_t)start_y+1, (SSD1306_COLOR)1);												
		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+2, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+3, (uint8_t)start_y+2, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+4, (uint8_t)start_y+2, (SSD1306_COLOR)1);						
		ssd1306_DrawPixel((uint8_t)start_x+3, (uint8_t)start_y+3, (SSD1306_COLOR)1);

}

void triangle_up(uint8_t start_x, uint8_t start_y)
{
//		ssd1306_DrawPixel((uint8_t)start_x, (uint8_t)start_y+3, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y+3, (SSD1306_COLOR)1);

//		ssd1306_DrawPixel((uint8_t)start_x+5, (uint8_t)start_y+3, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x+6, (uint8_t)start_y+3, (SSD1306_COLOR)1);	

//		ssd1306_DrawPixel((uint8_t)start_x+6, (uint8_t)start_y+4, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x+7, (uint8_t)start_y+4, (SSD1306_COLOR)1);	
	
		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y+2, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+2, (SSD1306_COLOR)1);

		ssd1306_DrawPixel((uint8_t)start_x+4, (uint8_t)start_y+2, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+5, (uint8_t)start_y+2, (SSD1306_COLOR)1);												
	
		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+1, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+3, (uint8_t)start_y+1, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+4, (uint8_t)start_y+1, (SSD1306_COLOR)1);					
	
		ssd1306_DrawPixel((uint8_t)start_x+3, (uint8_t)start_y, (SSD1306_COLOR)1);	
}

void triangle_right(uint8_t start_x, uint8_t start_y)
{
	
//		ssd1306_DrawPixel((uint8_t)start_x, (uint8_t)start_y, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x, (uint8_t)start_y+1, (SSD1306_COLOR)1);
////		ssd1306_DrawPixel(start_x, start_y+2, 1);
////		ssd1306_DrawPixel(start_x, start_y+3, 1);
////		ssd1306_DrawPixel(start_x, start_y+4, 1);
//		ssd1306_DrawPixel((uint8_t)start_x, (uint8_t)start_y+5, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x, (uint8_t)start_y+6, (SSD1306_COLOR)1);		
		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y+1, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y+2, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel(start_x+1, start_y+3, 1);
		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y+4, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y+5, (SSD1306_COLOR)1);		
		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+2, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+3, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+4,(SSD1306_COLOR) 1);		
		ssd1306_DrawPixel((uint8_t)start_x+3, (uint8_t)start_y+3, (SSD1306_COLOR)1);	
}

void triangle_left(uint8_t start_x, uint8_t start_y)
{
	
//		ssd1306_DrawPixel((uint8_t)start_x+3, (uint8_t)start_y, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x+3, (uint8_t)start_y+1, (SSD1306_COLOR)1);
////		ssd1306_DrawPixel(start_x+3, start_y+2, 1);
////		ssd1306_DrawPixel(start_x+3, start_y+3, 1);
////		ssd1306_DrawPixel(start_x+3, start_y+4, 1);
//		ssd1306_DrawPixel((uint8_t)start_x+3, (uint8_t)start_y+5, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel((uint8_t)start_x+3, (uint8_t)start_y+6, (SSD1306_COLOR)1);		
		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+1, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+2, (SSD1306_COLOR)1);
//		ssd1306_DrawPixel(start_x+2, start_y+3, 1);
		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+4, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+2, (uint8_t)start_y+5, (SSD1306_COLOR)1);		
		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y+2, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y+3, (SSD1306_COLOR)1);
		ssd1306_DrawPixel((uint8_t)start_x+1, (uint8_t)start_y+4, (SSD1306_COLOR)1);		
		ssd1306_DrawPixel((uint8_t)start_x, (uint8_t)start_y+3, (SSD1306_COLOR)1);	
}



void rombus(uint8_t start_x, uint8_t start_y)
{

		int i = 0;
    for (int y = start_y; y < start_y+9; y++)
    {
        for (int x = start_x; x < start_x+9; x++)
        {
            if (x <= start_x+4 + i && x >= start_x+4 - i) ssd1306_DrawPixel(x,y,(SSD1306_COLOR)1);
            else ssd1306_DrawPixel(x,y,(SSD1306_COLOR)0);
        }
        
        i = y >= start_y+4 ? --i : ++i;
    }
	
}

void horizont_line(uint8_t start_x, uint8_t start_y)
{
	for (int x = start_x; x < start_x+53; x++)
	{
		ssd1306_DrawPixel(x,start_y,(SSD1306_COLOR)1);	
		ssd1306_DrawPixel(x,start_y+1,(SSD1306_COLOR)1);
	}
}

void check_logo()
{
		
	for (int y = 0; y < 990; y+=55)
	{
		for (int x = 0; x < 55; x++)		
		{
				if (tik_logo2[x+y] == 0) ssd1306_DrawPixel(x+5, y/55+12, (SSD1306_COLOR)0);
		}
	}
	
}

void logo()
{
		
	for (int y = 0; y < 990; y+=55)
	{
		for (int x = 0; x < 55; x++)		
		{
				if (tik_logo2[x+y] == 0) ssd1306_DrawPixel(x+5, y/55+12, (SSD1306_COLOR)1);
		}
	}
	
}



