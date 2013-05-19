//
// Basic LED-blink for the MBED.
//
#include <stdint.h>
#include <stdio.h>
#include <string.h>
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

typedef enum {
  IDLE,
  ALIGN,
  PULSE_DOWN,
  PULSE_UP
} state_t;

volatile uint32_t systick_counter = 0;

volatile state_t state = IDLE;

uint32_t pulse_width = 100000000;
uint32_t pulse_misses = 0;

char lcd_buf[32];

void pwm_init(uint32_t period, uint32_t pulse);

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

    if(buttons & BUTTON1) {
      lcd_putc(0xFE); lcd_putc(0x1);
      lcd_puts("align");
      state = ALIGN;
    } else if(buttons & BUTTON2) {
      pulse_width = 100000000;
      lcd_putc(0xFE); lcd_putc(0x1);
      lcd_puts("Starting");
      pwm_init(pulse_width * 2, pulse_width);
      state = PULSE_DOWN;
    }
  }
}

void pwm_init(uint32_t period, uint32_t pulse) {

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

  // Stop on MR0
  LPC_PWM1->MCR = (1 << 2);

  // PWM1 output enabled
  LPC_PWM1->PCR = (1 << 9);

  // 50% duty cycle, 1 second period
  LPC_PWM1->MR0 = period;
  LPC_PWM1->MR1 = pulse;

  // M0 and M1 latch
  LPC_PWM1->LER = (1 << 0) | (1 << 1);

  // Enable PWM mode
  LPC_PWM1->TCR = (1 << 0) | (1 << 3);
}

static volatile uint8_t pulse_received = 0;

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
  LPC_GPIO2->FIOSET |= (1 << 0);

  lcd_init();

  // Clear display
  lcd_putc(0xFE);
  lcd_putc(0x1);

  lcd_puts("LaserSpeed v0.1");

  // Enable GPIO 2.0 interrupt
  LPC_GPIOINT->IO0IntClr |= (1 << DETECT_PIN);
  LPC_GPIOINT->IO0IntEnR |= (1 << DETECT_PIN);
  NVIC_EnableIRQ(EINT3_IRQn);

  delay_ms(1000);

  for(;;) {

    memset(lcd_buf, ' ', sizeof(lcd_buf));

    if(0 == (systick_counter & 0x1FF)) {
      // Toggle LED
      LPC_GPIO1->FIOPIN ^= (1 << LED4_PIN);
    }

    // Button processing
    process_buttons();

    switch(state) {
      case IDLE: 
        // Do nothing
        break;

      case ALIGN: 
        // Set pin as GPIO
        LPC_PINCON->PINSEL4 &= ~(1 << 0);
        LPC_GPIO2->FIOCLR |= (1 << 0);

        if(LPC_GPIO0->FIOPIN & (1 << DETECT_PIN)) {

          // Clear display
          lcd_putc(0xFE);
          lcd_putc(0x1);

          lcd_puts("aligned!");

          delay_ms(500);
          LPC_GPIO2->FIOSET |= (1 << 0);

          state = IDLE;
        }

        break;

      case PULSE_DOWN:
        // If pulse is detected, half the pulse width and try again
        if(pulse_received && !(LPC_PWM1->TCR & (1 << 0))) {
          pulse_received = 0;
          pulse_misses = 0;
          
          lcd_putc(0xFE);
          lcd_putc(0x1);
          sprintf(lcd_buf, "%d us", pulse_width / 100);
          lcd_puts(lcd_buf);

          pulse_width >>= 1;
          
          // Turn off LED
          LPC_PINCON->PINSEL4 &= ~(1 << 0);
          LPC_GPIO2->FIOSET |= (1 << 0);

          pwm_init(pulse_width * 2, pulse_width);
        } else if(!(LPC_PWM1->TCR & (1 << 0))) {
          pulse_misses++;
          if(pulse_misses > 1000) {
            // Looks like no pulse is coming, let's start searching back up
            state = PULSE_UP;
          }
        }

        break;

      case PULSE_UP:
      // If pulse is detected, half the pulse width and try again
        if(!pulse_received && !(LPC_PWM1->TCR & (1 << 0))) {
          pulse_received = 0;
          pulse_misses = 0;
          
          lcd_putc(0xFE);
          lcd_putc(0x1);
          sprintf(lcd_buf, "%d us", pulse_width / 100);
          lcd_puts(lcd_buf);

          pulse_width += 100;
          
          // Turn off LED
          LPC_PINCON->PINSEL4 &= ~(1 << 0);
          LPC_GPIO2->FIOSET |= (1 << 0);

          pwm_init(pulse_width * 2, pulse_width);
        } else if(pulse_received) {
          lcd_putc(0xFE);
          lcd_putc(0x1);
          sprintf(lcd_buf, "%d us *", pulse_width / 100);
          lcd_puts(lcd_buf);
          state = IDLE;
        }
        
        break;
    }

    __WFI();      
  }
}

void EINT3_IRQHandler(void) {
  LPC_GPIOINT->IO0IntClr |= (1 << DETECT_PIN);
  pulse_received = 1;
}
