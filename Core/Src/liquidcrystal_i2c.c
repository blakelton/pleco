/*
 * liquidcrystal_i2c.c
 *
 *  Created on: Dec 11, 2024
 *      Author: bazue
 */
#include "liquidcrystal_i2c.h"
#include "cmsis_os.h"
#include <string.h>

// Private function prototypes
static void lcd_init_priv(LiquidCrystal_I2C *lcd);
static void lcd_expanderWrite(LiquidCrystal_I2C *lcd, uint8_t _data);
static void lcd_pulseEnable(LiquidCrystal_I2C *lcd, uint8_t _data);
static void lcd_write4bits(LiquidCrystal_I2C *lcd, uint8_t value);
static void lcd_send(LiquidCrystal_I2C *lcd, uint8_t value, uint8_t mode);

// Function implementations

void lcd_init(LiquidCrystal_I2C *lcd, I2C_HandleTypeDef *hi2c, uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows) {
    lcd->hi2c = hi2c;
    lcd->Addr = lcd_Addr << 1; // Shift address for STM32 HAL
    lcd->cols = lcd_cols;
    lcd->rows = lcd_rows;
    lcd->backlightval = LCD_BACKLIGHT;

    lcd_init_priv(lcd);
}

static void lcd_init_priv(LiquidCrystal_I2C *lcd) {
    lcd->displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    if (lcd->rows > 1) {
        lcd->displayfunction |= LCD_2LINE;
    }

    lcd->numlines = lcd->rows;

    // Wait for LCD to power up
    HAL_Delay(50);

    // Initialize the LCD
    lcd_expanderWrite(lcd, lcd->backlightval);
    HAL_Delay(1000);

    // Set to 4-bit mode
    lcd_write4bits(lcd, 0x03 << 4);
    HAL_Delay(5);
    lcd_write4bits(lcd, 0x03 << 4);
    HAL_Delay(5);
    lcd_write4bits(lcd, 0x03 << 4);
    HAL_Delay(1);
    lcd_write4bits(lcd, 0x02 << 4);

    // Function set
    lcd_command(lcd, LCD_FUNCTIONSET | lcd->displayfunction);

    // Display control
    lcd->displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    lcd_display(lcd);

    // Clear display
    lcd_clear(lcd);

    // Entry mode set
    lcd->displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    lcd_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);

    lcd_home(lcd);
}

void lcd_clear(LiquidCrystal_I2C *lcd) {
    lcd_command(lcd, LCD_CLEARDISPLAY);
    HAL_Delay(2);
}

void lcd_home(LiquidCrystal_I2C *lcd) {
    lcd_command(lcd, LCD_RETURNHOME);
    HAL_Delay(2);
}

void lcd_setCursor(LiquidCrystal_I2C *lcd, uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row >= lcd->numlines) {
        row = lcd->numlines - 1;
    }
    lcd_command(lcd, LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd_noDisplay(LiquidCrystal_I2C *lcd) {
    lcd->displaycontrol &= ~LCD_DISPLAYON;
    lcd_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}

void lcd_display(LiquidCrystal_I2C *lcd) {
    lcd->displaycontrol |= LCD_DISPLAYON;
    lcd_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}

void lcd_noBlink(LiquidCrystal_I2C *lcd) {
    lcd->displaycontrol &= ~LCD_BLINKON;
    lcd_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}

void lcd_blink(LiquidCrystal_I2C *lcd) {
    lcd->displaycontrol |= LCD_BLINKON;
    lcd_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}

void lcd_noCursor(LiquidCrystal_I2C *lcd) {
    lcd->displaycontrol &= ~LCD_CURSORON;
    lcd_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}

void lcd_cursor(LiquidCrystal_I2C *lcd) {
    lcd->displaycontrol |= LCD_CURSORON;
    lcd_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}

void lcd_scrollDisplayLeft(LiquidCrystal_I2C *lcd) {
    lcd_command(lcd, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void lcd_scrollDisplayRight(LiquidCrystal_I2C *lcd) {
    lcd_command(lcd, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void lcd_leftToRight(LiquidCrystal_I2C *lcd) {
    lcd->displaymode |= LCD_ENTRYLEFT;
    lcd_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);
}

void lcd_rightToLeft(LiquidCrystal_I2C *lcd) {
    lcd->displaymode &= ~LCD_ENTRYLEFT;
    lcd_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);
}

void lcd_autoscroll(LiquidCrystal_I2C *lcd) {
    lcd->displaymode |= LCD_ENTRYSHIFTINCREMENT;
    lcd_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);
}

void lcd_noAutoscroll(LiquidCrystal_I2C *lcd) {
    lcd->displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
    lcd_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);
}

void lcd_createChar(LiquidCrystal_I2C *lcd, uint8_t location, uint8_t charmap[]) {
    location &= 0x7;
    lcd_command(lcd, LCD_SETCGRAMADDR | (location << 3));
    for (int i = 0; i < 8; i++) {
        lcd_write(lcd, charmap[i]);
    }
}

void lcd_backlight(LiquidCrystal_I2C *lcd) {
    lcd->backlightval = LCD_BACKLIGHT;
    lcd_expanderWrite(lcd, 0);
}

void lcd_noBacklight(LiquidCrystal_I2C *lcd) {
    lcd->backlightval = LCD_NOBACKLIGHT;
    lcd_expanderWrite(lcd, 0);
}

void lcd_write(LiquidCrystal_I2C *lcd, uint8_t value) {
    lcd_send(lcd, value, Rs);
}

void lcd_command(LiquidCrystal_I2C *lcd, uint8_t value) {
    lcd_send(lcd, value, 0);
}

void lcd_print(LiquidCrystal_I2C *lcd, const char *str) {
    while (*str) {
        lcd_write(lcd, (uint8_t)(*str));
        str++;
    }
}

// Private functions

static void lcd_send(LiquidCrystal_I2C *lcd, uint8_t value, uint8_t mode) {
    uint8_t highnib = value & 0xF0;
    uint8_t lownib = (value << 4) & 0xF0;
    lcd_write4bits(lcd, highnib | mode);
    lcd_write4bits(lcd, lownib | mode);
}

static void lcd_write4bits(LiquidCrystal_I2C *lcd, uint8_t value) {
    lcd_expanderWrite(lcd, value);
    lcd_pulseEnable(lcd, value);
}

static void lcd_expanderWrite(LiquidCrystal_I2C *lcd, uint8_t _data) {
    uint8_t data = _data | lcd->backlightval;
    HAL_I2C_Master_Transmit(lcd->hi2c, lcd->Addr, &data, 1, HAL_MAX_DELAY);
}

static void lcd_pulseEnable(LiquidCrystal_I2C *lcd, uint8_t _data) {
    lcd_expanderWrite(lcd, _data | En);
    HAL_Delay(1);
    lcd_expanderWrite(lcd, _data & ~En);
    HAL_Delay(1);
}
