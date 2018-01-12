/*
 * ssd1306.h
 *
 *  Created on: 10.01.2018
 *      Author: Mateusz
 */

#ifndef SSD1306_H_
#define SSD1306_H_

#include "stm32f1xx.h"

#define SSD1306_I2C_ADRESS 0x78

#define SSD1306_LCDWIDTH 128
#define SSD1306_LCDHEIGHT 64


#define SSD1306_REFRESH_MIN 0x80
#define SSD1306_REFRESH_MID 0xB0
#define SSD1306_REFRESH_MAX 0xF0

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0xB0

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR 0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

//extern uint8_t ssd1306_buf[SSD1306_LCDWIDTH * (SSD1306_LCDHEIGHT/8)];



void ssd1306_initialize();
void i2c_write_buf(uint8_t addres, uint8_t adr, uint8_t *buf, uint16_t size);
void i2c_write(uint8_t address, uint8_t adr, uint8_t data);
void ssd1306_task();
void ssd1306_display();
void ssd1306_clear_screen();
void ssd1306_draw_string(const char *string, uint8_t x, uint8_t y);
void ssd1306_command(uint8_t command);


#endif /* SSD1306_H_ */
