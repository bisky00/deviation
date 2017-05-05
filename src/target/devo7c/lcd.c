/*
    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Deviation is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include "common.h"
#include "gui/gui.h"

#define CS_HI() gpio_set(GPIOB, GPIO0)
#define CS_LO() gpio_clear(GPIOB, GPIO0)
#define CMD_MODE() gpio_clear(GPIOC,GPIO5)
#define DATA_MODE() gpio_set(GPIOC,GPIO5)

#define RST_HI() gpio_set(GPIOB, GPIO1)
#define RST_LO() gpio_clear(GPIOB, GPIO1)

//The screen is 129 characters, but we'll only expoise 128 of them
#define PHY_LCD_WIDTH 128
#define LCD_PAGES 8
static u8 img[PHY_LCD_WIDTH * LCD_PAGES];
static u8 dirty[PHY_LCD_WIDTH];
static u16 xstart, xend;  // After introducing logical view for devo10, the coordinate can be >= 5000
static u16 xpos, ypos;
static s8 dir;

void LCD_Cmd(unsigned cmd) {
    CMD_MODE();
    CS_LO();
    spi_xfer(SPI1, cmd);
    CS_HI();
}

void LCD_Data(unsigned cmd) {
    DATA_MODE();
    CS_LO();
    spi_xfer(SPI1, cmd);
    CS_HI();
}

void lcd_display(uint8_t on)
{
    LCD_Cmd(0xAE | (on ? 1 : 0));
}

void lcd_set_page_address(uint8_t page)
{
    LCD_Cmd(0xB0 | (page & 0x07));
}

void lcd_set_column_address(uint8_t column)
{
    LCD_Cmd(0x10 | ((column >> 4) & 0x0F));  //MSB
    LCD_Cmd(column & 0x0F);                  //LSB
}

void lcd_set_start_line(int line)
{
  LCD_Cmd((line & 0x3F) | 0x40);
}

void LCD_Contrast(unsigned contrast)
{
    (void)contrast;
/*
    //int data = 0x20 + contrast * 0xC / 10;
    LCD_Cmd(0x81);
    int c = contrast * 12 + 76; //contrast should range from ~72 to ~200
    LCD_Cmd(c);
*/
}

void LCD_Init()
{
    //Initialization is mostly done in SPI Flash
    //Setup CS = B.0, Data/Control = C.5, RESET = B.1
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
  	RST_HI();
//    volatile int i = 0x8000;
//    while(i) i--;
    RST_LO();
//    i = 0x8000;
//    while(i) i--;
  	RST_HI();
    LCD_Cmd(0xae);//--turn off oled panel
    LCD_Cmd(0x00);//---set low column address
    LCD_Cmd(0x10);//---set high column address
    LCD_Cmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    LCD_Cmd(0x81);//--set contrast control register
    LCD_Cmd(0xcf); // Set SEG Output Current Brightness
    LCD_Cmd(0xa1);//--Set SEG/Column Mapping
    LCD_Cmd(0xc8);//Set COM/Row Scan Direction
    LCD_Cmd(0xa6);//--set normal display
    LCD_Cmd(0xa8);//--set multiplex ratio(1 to 64)
    LCD_Cmd(0x3f);//--1/64 duty
    LCD_Cmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    LCD_Cmd(0x00);//-not offset
    LCD_Cmd(0xd5);//--set display clock divide ratio/oscillator frequency
    LCD_Cmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
    LCD_Cmd(0xd9);//--set pre-charge period
    LCD_Cmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    LCD_Cmd(0xda);//--set com pins hardware configuration
    LCD_Cmd(0x12);
    LCD_Cmd(0xdb);//--set vcomh
    LCD_Cmd(0x40);//Set VCOM Deselect Level
    LCD_Cmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
    LCD_Cmd(0x02);//
    LCD_Cmd(0x8d);//--set Charge Pump enable/disable
    LCD_Cmd(0x14);//--set(0x10) disable
    LCD_Cmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
    LCD_Cmd(0xa6);// Disable Inverse Display On (0xa6/a7)
    LCD_Cmd(0xaf);//--turn on oled panel
    memset(img, 0, sizeof(img));
    memset(dirty, 0, sizeof(dirty));
}

void LCD_Clear(unsigned int val)
{
    val = (val & 0xFF) ? 0xff : 0x00;
    memset(img, val, sizeof(img));
    memset(dirty, 0xFF, sizeof(dirty));
}

void LCD_DrawStart(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, enum DrawDir _dir)
{
    if (_dir == DRAW_SWNE) {
        ypos = y1;  // bug fix: must do it this way to draw bmp
        dir = -1;
    } else {
        ypos = y0;
        dir = 1;
    }
    xstart = x0;
    xend = x1;
    xpos = x0;
}
/* Screen coordinates are as follows:
 * (128, 32)   ....   (0, 32)
 *   ...       ....     ...
 * (128, 63)   ....   (0, 63)
 * (128, 0)    ....   (0, 0)
 *   ...       ....     ...
 * (128, 31)   ....   (0, 31)
 */
void LCD_DrawStop(void)
{
    int col = 0;
    int p, c;
    for (p = 0; p < LCD_PAGES; p++) {
        int init = 0;
        for (c = 0; c < PHY_LCD_WIDTH; c++) {
            if(dirty[c] & (1 << p)) {
                if(! init) {
                    lcd_set_page_address(p);
                    lcd_set_column_address(c);
                } else if(col+1 != c) {
                    lcd_set_column_address(c);
                }
                LCD_Data(img[p * PHY_LCD_WIDTH + c]);
                col = c;
            }
        }
    }
    memset(dirty, 0, sizeof(dirty));
}

void LCD_DrawPixel(unsigned int color)
{
	if (xpos < LCD_WIDTH && ypos < LCD_HEIGHT) {	// both are unsigned, can not be < 0
		int y = ypos;
		int x = xpos;
		int ycol = y / 8;
		int ybit = y & 0x07;
        if (color) {
            img[ycol * PHY_LCD_WIDTH + x] |= 1 << ybit;
        } else {
            img[ycol * PHY_LCD_WIDTH + x] &= ~(1 << ybit);
        }
        dirty[x] |= 1 << ycol;
    }
	// this must be executed to continue drawing in the next row
    xpos++;
    if (xpos > xend) {
        xpos = xstart;
        ypos += dir;
    }
}

void LCD_DrawPixelXY(unsigned int x, unsigned int y, unsigned int color)
{
    xpos = x;
    ypos = y;
    LCD_DrawPixel(color);
}
