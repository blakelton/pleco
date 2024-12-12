/*
 * liquidcrystal_i2c.h
 *
 *  Created on: Dec 11, 2024
 *      Author: bazue
 */

#ifndef INC_LIQUIDCRYSTAL_I2C_H_
#define INC_LIQUIDCRYSTAL_I2C_H_

#include "stm32l4xx_hal.h"
#include "i2c.h"           // Include the I2C handle

// LCD Commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// Flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// Flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// Flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// Flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// Flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

// Enable bit
#define En 0x04
// Read/Write bit
#define Rw 0x02
// Register select bit
#define Rs 0x01

typedef struct {
    I2C_HandleTypeDef *hi2c; // Pointer to I2C handle
    uint8_t Addr;            // I2C address of the LCD
    uint8_t displayfunction;
    uint8_t displaycontrol;
    uint8_t displaymode;
    uint8_t numlines;
    uint8_t cols;
    uint8_t rows;
    uint8_t backlightval;
} LiquidCrystal_I2C;

// Public functions
void lcd_init(LiquidCrystal_I2C *lcd, I2C_HandleTypeDef *hi2c, uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows);
void lcd_clear(LiquidCrystal_I2C *lcd);
void lcd_home(LiquidCrystal_I2C *lcd);
void lcd_noDisplay(LiquidCrystal_I2C *lcd);
void lcd_display(LiquidCrystal_I2C *lcd);
void lcd_noBlink(LiquidCrystal_I2C *lcd);
void lcd_blink(LiquidCrystal_I2C *lcd);
void lcd_noCursor(LiquidCrystal_I2C *lcd);
void lcd_cursor(LiquidCrystal_I2C *lcd);
void lcd_scrollDisplayLeft(LiquidCrystal_I2C *lcd);
void lcd_scrollDisplayRight(LiquidCrystal_I2C *lcd);
void lcd_leftToRight(LiquidCrystal_I2C *lcd);
void lcd_rightToLeft(LiquidCrystal_I2C *lcd);
void lcd_autoscroll(LiquidCrystal_I2C *lcd);
void lcd_noAutoscroll(LiquidCrystal_I2C *lcd);
void lcd_createChar(LiquidCrystal_I2C *lcd, uint8_t location, uint8_t charmap[]);
void lcd_setCursor(LiquidCrystal_I2C *lcd, uint8_t col, uint8_t row);
void lcd_write(LiquidCrystal_I2C *lcd, uint8_t value);
void lcd_command(LiquidCrystal_I2C *lcd, uint8_t value);
void lcd_backlight(LiquidCrystal_I2C *lcd);
void lcd_noBacklight(LiquidCrystal_I2C *lcd);
void lcd_print(LiquidCrystal_I2C *lcd, const char *str);

#endif /* INC_LIQUIDCRYSTAL_I2C_H_ */
