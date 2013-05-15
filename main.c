//
// Basic LED-blink for the MBED.
//
#include "LPC17xx.h"
#include <stdint.h>

#define LED4_PIN (22)

void my_putc(char c) {
  while(!(LPC_UART0->LSR & (1 << 6)));
  LPC_UART0->THR = c; 
}

void my_puts(char * string) {
  char *p_char = string;
    while(*p_char) {
      my_putc(*p_char++);
    }
}

int main() {
  
  SystemInit();
  
  // Setup P1.23 as output
  LPC_GPIO0->FIODIR |= (1 << LED4_PIN);

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

  // Clear display
  my_putc(0xFE);
  my_putc(0x1);

  uint8_t num = 0;

  for(;;) {
    for(uint32_t delay = 0; delay < 10000000; delay++) {
       __asm("NOP");
    }
    
    // Turn LED ON
    LPC_GPIO0->FIOSET = (1 << LED4_PIN);
    
    for(uint32_t delay = 0; delay < 10000000; delay++) {
       __asm("NOP");
    }

    // Clear display
    my_putc(0xFE);
    my_putc(0x1);

    my_puts("hello");
    my_putc('0' + num++);

    if(num > 9) {
      num = 0;
    }
    
    // Turn LED OFF
    LPC_GPIO0->FIOCLR = (1 << LED4_PIN);
  }
}
