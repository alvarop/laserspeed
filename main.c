//
// Basic LED-blink for the MBED.
//
#include <stdint.h>
#include "LPC17xx.h"
#include "lcd.h"

#define LED4_PIN (22)

#define BTN1_PIN (27)
#define BTN2_PIN (28)

#define DEBOUNCE_MS (100)
#define BUTTON1     (1 << 0) 
#define BUTTON2     (1 << 1)

volatile uint32_t systick_counter = 0;

void SysTick_Handler (void) {
  systick_counter++;
}

void delay_ms(uint32_t delay) {
  uint32_t limit = systick_counter + delay;

  // WARNING: not taking overflow into account!
  while(systick_counter < limit) {
    __WFI();
  }
}

void process_buttons() {
  static uint8_t button1_count = 0;
  static uint8_t button2_count = 0;
  static uint8_t last_buttons = 0;
  static uint8_t buttons = 0;

  if((LPC_GPIO0->FIOPIN & (1 << BTN1_PIN)) == 0) {
    button1_count++;
    if(button1_count > DEBOUNCE_MS) {
      buttons |= BUTTON1;
    }
  } else {
    button1_count = 0;
    buttons &= ~BUTTON1;
  }

  if((LPC_GPIO0->FIOPIN & (1 << BTN2_PIN)) == 0) {
    button2_count++;
    if(button2_count > DEBOUNCE_MS) {
      buttons |= BUTTON2;
    }
  } else {
    button2_count = 0;
    buttons &= ~BUTTON2;
  }

  if(last_buttons != buttons) {
    last_buttons = buttons;
    lcd_putc(0xFE); lcd_putc(0x1);

    if(buttons & BUTTON1) {
      lcd_puts("start");
    } else if(buttons & BUTTON2) {
      lcd_puts("stop");
    }
  }
}

int main() {
  
  SystemInit();

  SysTick_Config(SystemCoreClock/1000 - 1); // Generate interrupt each 1 ms
  
  // Setup P0.22 as output
  LPC_GPIO0->FIODIR |= (1 << LED4_PIN);

  // Buttons as inputs
  LPC_GPIO0->FIODIR &= ~(1 << BTN1_PIN);
  LPC_GPIO0->FIODIR &= ~(1 << BTN2_PIN);

  lcd_init();

  // Clear display
  lcd_putc(0xFE);
  lcd_putc(0x1);

  lcd_puts("LaserSpeed v0.1");

  delay_ms(1000);

  for(;;) {
    if(0 == (systick_counter & 0x1FF)) {
      // Toggle LED
      LPC_GPIO0->FIOPIN ^= (1 << LED4_PIN);
    }

    // Button processing
    process_buttons();

    __WFI();      
  }
}
