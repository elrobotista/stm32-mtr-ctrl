#include <stdint.h>
#include <stdbool.h>
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/cm3/systick.h"
#include "libopencm3/cm3/nvic.h"

#define SYSTICK_FRQ (1000)

/* Lowest priority. As defined by the upper nibble. */
/* LSB nibble is don't care. */
#define SYSTICK_PRIO (0xF0)

#define GPIO_PORT_LED1 (GPIOA)
#define GPIO_PIN_LED1 (GPIO5)
#define RCC_PORT_LED1 (RCC_GPIOA)

void hw_init(void);
void gpio_init(void);
void clock_init(void);
void pwm_init(void);

int main(void) {
  hw_init();
  while(true);
  return 0;
}

void sys_tick_handler(void) {
  static uint32_t tmr;
  if(tmr++ >= 1e3) {
    tmr = 0;
    gpio_toggle(GPIO_PORT_LED1, GPIO_PIN_LED1);
  }
}

void hw_init(void) {
  clock_init();
  gpio_init();
}

void gpio_init(void) {
  rcc_periph_clock_enable(RCC_PORT_LED1);
  gpio_mode_setup(
    GPIO_PORT_LED1,
    GPIO_MODE_OUTPUT,
    GPIO_PUPD_NONE,
    GPIO_PIN_LED1
  );
  gpio_set_output_options(
    GPIO_PORT_LED1,
    GPIO_OTYPE_PP,
    GPIO_OSPEED_2MHZ,
    GPIO_PIN_LED1
  );
}

void clock_init(void) {
  bool ret;
  rcc_clock_setup_pll(&rcc_hse8mhz_configs[RCC_CLOCK_HSE8_72MHZ]);
  ret = systick_set_frequency((uint32_t)SYSTICK_FRQ, rcc_ahb_frequency);
  if(ret == false) {
    /* Do nothing for now. Will add error traing later on. */
  }
  nvic_set_priority(NVIC_SYSTICK_IRQ, SYSTICK_PRIO);
  systick_clear();
  systick_counter_enable();
  systick_interrupt_enable();
}

void pwm_init(void) {
  ;
}
