#include <stdint.h>
#include <stdbool.h>
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/cm3/systick.h"
#include "libopencm3/cm3/nvic.h"

#define SYSTICK_FRQ (1000)

/* Lowest priority. As defined by the upper nibble. */
/* LSB nibble is don't care. */
#define SYSTICK_PRIO (0xF0)

#define GPIO_PORT_LED1 (GPIOA)
#define GPIO_PIN_LED1 (GPIO5)
#define RCC_PORT_LED1 (RCC_GPIOA)

/* PWM definition / macros. */
#define PWM_PORT_X (GPIOA)
#define PWM_RCC_PORT (RCC_GPIOA)
#define PWM_GPIO_U (GPIO8)
#define PWM_GPIO_V (GPIO9)
#define PWM_GPIO_W (GPIO10)

/* As defined by APB2 clock. */
#define PWM_TMR_CLK_HZ (72000000)
#define PWM_TMR_FRQ_HZ (10000)
#define PWM_TMR_PSC (0)
#define PWM_TMR_PRD (((PWM_TMR_CLK_HZ / (2 * PWM_TMR_FRQ_HZ)) / (PWM_TMR_PSC + 1)) - 1)
#if (PWM_TMR_PRD > 65535) || (PWM_TMR_PRD < 1)
#error("Invalid PWM config: ARR register out of bounds.")
#endif

static void hw_init(void);
static void gpio_init(void);
static void clock_init(void);
static void pwm_init(void);

int main(void) {
  hw_init();
  while(true);
  return 0;
}

void sys_tick_handler(void) {
  static uint32_t tmr = 0;
  if(tmr++ >= 1e3) {
    tmr = 0;
    gpio_toggle(GPIO_PORT_LED1, GPIO_PIN_LED1);
    timer_set_oc_value(TIM1, TIM_OC1, 10);
    timer_set_oc_value(TIM1, TIM_OC2, 10);
    timer_set_oc_value(TIM1, TIM_OC3, 10);
  }
}

static void hw_init(void) {
  clock_init();
  gpio_init();
  pwm_init();
}

static void gpio_init(void) {
  rcc_periph_clock_enable(RCC_PORT_LED1);
  rcc_periph_clock_enable(PWM_RCC_PORT);
  /* LED output configuration. */
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
  /* PWM output configuration. */
  gpio_mode_setup(
    PWM_PORT_X,
    GPIO_MODE_AF,
    GPIO_PUPD_NONE,
    (PWM_GPIO_U | PWM_GPIO_V | PWM_GPIO_W)
  );
  gpio_set_output_options(
    PWM_PORT_X,
    GPIO_OTYPE_PP,
    GPIO_OSPEED_50MHZ,
    (PWM_GPIO_U | PWM_GPIO_V | PWM_GPIO_W)
  );
  gpio_set_af(
    PWM_PORT_X,
    GPIO_AF6,
    (PWM_GPIO_U | PWM_GPIO_V | PWM_GPIO_W)
  );
  gpio_port_config_lock(
    PWM_PORT_X,
    (PWM_GPIO_U | PWM_GPIO_V | PWM_GPIO_W)
  );
}

static void clock_init(void) {
  bool ret;
  rcc_clock_setup_pll(&rcc_hse8mhz_configs[RCC_CLOCK_HSE8_72MHZ]);
  (void)systick_set_frequency(SYSTICK_FRQ, rcc_ahb_frequency);
  nvic_set_priority(NVIC_SYSTICK_IRQ, SYSTICK_PRIO);
  systick_clear();
  systick_counter_enable();
  systick_interrupt_enable();
}

static void pwm_init(void) {
  rcc_periph_clock_enable(RCC_TIM1);
  timer_set_mode(
    TIM1,
    TIM_CR1_CKD_CK_INT_MUL_2,
    /* Center aglined mode: output compare interrupt flags set only */
    /* when counter is counting down. */
    TIM_CR1_CMS_CENTER_1,
    TIM_CR1_DIR_UP
  );
  /* Update reload value is updated immediately without buffering. */
  timer_disable_preload(TIM1);

  /* One pulse mode is turnde off. */
  timer_continuous_mode(TIM1);
  timer_set_prescaler(TIM1, PWM_TMR_PSC);
  timer_set_repetition_counter(TIM1, 0);
  timer_set_period(TIM1, PWM_TMR_PRD);

  /* Deadtime configuration. */
  timer_set_deadtime(TIM1, 0);

  /* Break feature configuration. */
  timer_set_disabled_off_state_in_run_mode(TIM1);
  timer_set_disabled_off_state_in_idle_mode(TIM1);
  timer_disable_break(TIM1);

  /* Should not matter as break feature is disabled. */
  timer_set_break_polarity_low(TIM1);

  /* PWM must be re-enabled by software in the case of break event. */
  /* This should not matter as break feature is disabled. */
  timer_disable_break_automatic_output(TIM1);

  /* No register locked against write op. */
  timer_set_break_lock(TIM1, TIM_BDTR_LOCK_OFF);

  /* OC1 configuration. */
  timer_disable_oc_output(TIM1, TIM_OC1);
  timer_set_oc_polarity_high(TIM1, TIM_OC1);

  /* Do not use external trigger (slave mode). */
  timer_disable_oc_clear(TIM1, TIM_OC1);
  /* timer_enable_oc_preload(TIM1, TIM_OC1); */

  /* Output only dependant on counter. */
  timer_set_oc_slow_mode(TIM1, TIM_OC1);
  timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
  timer_set_oc_value(TIM1, TIM_OC1, 0);

  /* OC2 configuration. */
  timer_disable_oc_output(TIM1, TIM_OC2);
  timer_set_oc_polarity_high(TIM1, TIM_OC2);

  /* Do not use external trigger (slave mode). */
  timer_disable_oc_clear(TIM1, TIM_OC2);
  /* timer_enable_oc_preload(TIM1, TIM_OC2); */

  /* Output only dependant on counter. */
  timer_set_oc_slow_mode(TIM1, TIM_OC2);
  timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
  timer_set_oc_value(TIM1, TIM_OC2, 0);

  /* OC3 configuration. */
  timer_disable_oc_output(TIM1, TIM_OC3);
  timer_set_oc_polarity_high(TIM1, TIM_OC3);

  /* Do not use external trigger (slave mode). */
  timer_disable_oc_clear(TIM1, TIM_OC3);
  /* timer_enable_oc_preload(TIM1, TIM_OC3); */

  /* Output only dependant on counter. */
  timer_set_oc_slow_mode(TIM1, TIM_OC3);
  timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);
  timer_set_oc_value(TIM1, TIM_OC3, 0);

  /* Master out configuration. */
  timer_enable_oc_output(TIM1, TIM_OC1);
  timer_enable_oc_output(TIM1, TIM_OC2);
  timer_enable_oc_output(TIM1, TIM_OC3);

  /* Need to enable main output even if break / deadtime features are not used. */
  timer_enable_break_main_output(TIM1);
  timer_enable_counter(TIM1);
}
