//
// Basic LED-blink for the MBED.
//
#include <stdint.h>
#include "LPC17xx.h"
#include "lcd.h"

#define LASER_PIN (0)

#define DETECT_PIN (15)
#define LED4_PIN (18)

#define BTN1_PIN (18)
#define BTN2_PIN (17)

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

void pwm_init() {

  // PWM1 reset
  LPC_PWM1->TCR = (1 << 1);

  // Enable PWM1 power
  LPC_SC->PCONP |= (1 << 6);

  // PWM1 Clock
  LPC_SC->PCLKSEL0 = (1 << 12); // CCLK/1 = 100MHz

  // GPIO 2.0 -> PWM1.1
  LPC_PINCON->PINSEL4 |= (1 << 0);

  // No pull down on GPIO 2.0
  LPC_PINCON->PINMODE4 |= (2 << 0);

  // Reset on MR0
  LPC_PWM1->MCR = (1 << 1);

  // PWM1 output enabled
  LPC_PWM1->PCR = (1 << 9);

  // 50% duty cycle, 1 second period
  LPC_PWM1->MR0 = 100000000;
  LPC_PWM1->MR1 = 50000000;

  // M0 and M1 latch
  LPC_PWM1->LER = (1 << 0) | (1 << 1);

  // Enable PWM mode
  LPC_PWM1->TCR = (1 << 0) | (1 << 3);
}

int main() {
  
  SystemInit();

  SysTick_Config(SystemCoreClock/1000 - 1); // Generate interrupt each 1 ms
  
  // Setup LED as output
  LPC_GPIO1->FIODIR |= (1 << LED4_PIN);

  // Buttons as inputs
  LPC_GPIO0->FIODIR &= ~(1 << BTN1_PIN);
  LPC_GPIO0->FIODIR &= ~(1 << BTN2_PIN);

  // Detect as input
  LPC_GPIO0->FIODIR &= ~(1 << DETECT_PIN);

  // Laser control output
  LPC_GPIO2->FIODIR |= (1 << LASER_PIN);

  lcd_init();

  // Clear display
  lcd_putc(0xFE);
  lcd_putc(0x1);

  lcd_puts("LaserSpeed v0.1");

  pwm_init();

  delay_ms(1000);

  for(;;) {
    if(0 == (systick_counter & 0x1FF)) {
      // Toggle LED
      LPC_GPIO1->FIOPIN ^= (1 << LED4_PIN);
    }

    // Button processing
    process_buttons();

    if(LPC_GPIO0->FIOPIN & (1 << DETECT_PIN)) {
      // detect!
    }

    __WFI();      
  }
}
