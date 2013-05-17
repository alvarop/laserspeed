#include "lcd.h"
#include "LPC17xx.h"

void lcd_putc(char c) {
	while(!(LPC_UART2->LSR & (1 << 6)));
	LPC_UART2->THR = c; 
}

void lcd_puts(char * string) {
	char *p_char = string;
	while(*p_char) {
		lcd_putc(*p_char++);
	}
}

void lcd_init() {

	// Enable UART2 power
	LPC_SC->PCONP |= (1 << 24);

	// Setup UART2
	LPC_SC->PCLKSEL1 = (1 << 16); // CCLK/1 = 100MHz

	// 8, N, 1 -- Enable divisor latch access bit 
	LPC_UART2->LCR = (3 << 0) | (1 << 7);

	// 9600 * 16 = 153600
	// 100MHz/153600 =~ 651 = 0x28B
	LPC_UART2->DLL = 0x8B;
	LPC_UART2->DLM = 0x02;

	// Clear DLAB
	LPC_UART2->LCR &= ~(1 << 7);

	// baudrate = PCLK/(16 * (256 * DLM + DLL) * (1 + FDR.DIVADDVAL/FDR.MULVAL))

	// GPIO 0.2 -> TXD0
	LPC_PINCON->PINSEL0 |= (1 << 20);

	// Pull down on GPIO 0.2
	LPC_PINCON->PINMODE0 |= (2 << 20);
}
