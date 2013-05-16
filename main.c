//
// Basic LED-blink for the MBED.
//
#include <stdint.h>
#include "LPC17xx.h"
#include "lcd.h"

#define LED4_PIN (22)

volatile uint32_t systick_counter = 0;

void SysTick_Handler (void) {
  systick_counter++;
}

void delay_ms(uint32_t delay) {
  uint32_t limit = systick_counter + delay;

  // WARNING: not taking rollover into account!
  while(systick_counter < limit) {
    __WFI();
  }
}

int main() {
  
  SystemInit();

  SysTick_Config(SystemCoreClock/1000 - 1); // Generate interrupt each 1 ms
  
  // Setup P1.23 as output
  LPC_GPIO0->FIODIR |= (1 << LED4_PIN);

  lcd_init();

  // Clear display
  lcd_putc(0xFE);
  lcd_putc(0x1);

  lcd_puts("LaserSpeed v0.1");

  delay_ms(5000);

  uint8_t num = 0;

  for(;;) {
    if(0 == (systick_counter % 500)) {
      // Toggle LED
      LPC_GPIO0->FIOPIN ^= (1 << LED4_PIN);
    }

    __WFI();      
  }
}
