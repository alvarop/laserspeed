#include "lcd.h"
#include "LPC17xx.h"

void lcd_putc(char c) {
	while(!(LPC_UART0->LSR & (1 << 6)));
	LPC_UART0->THR = c; 
}

void lcd_puts(char * string) {
	char *p_char = string;
	while(*p_char) {
		lcd_putc(*p_char++);
	}
}

void lcd_init() {
	// Setup UART0
	LPC_SC->PCLKSEL0 = (1 << 6); // CCLK/1 = 100MHz

	// 8, N, 1 -- Enable divisor latch access bit 
	LPC_UART0->LCR = (3 << 0) | (1 << 7);

	// 9600 * 16 = 153600
	// 100MHz/153600 =~ 651 = 0x28B
	LPC_UART0->DLL = 0x8B;
	LPC_UART0->DLM = 0x02;

	// Clear DLAB
	LPC_UART0->LCR &= ~(1 << 7);

	// baudrate = PCLK/(16 * (256 * DLM + DLL) * (1 + FDR.DIVADDVAL/FDR.MULVAL))

	// GPIO 0.2 -> TXD0
	LPC_PINCON->PINSEL0 |= (1 << 4);

	// Pull down on GPIO 0.2
	LPC_PINCON->PINMODE0 |= (2 << 4);
}
