#ifndef __LCD_H__
#define __LCD_H__

#include <stdint.h>
#include "lcd.h"
#include "LPC17xx.h"

void lcd_putc(char c);
void lcd_puts(char * string);
void lcd_init();

#endif
