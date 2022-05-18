#include <stdint.h>
#include <stdbool.h>
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/stm32/adc.h"
#include "libopencm3/cm3/systick.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/cortex.h"


#define SYSTICK_FRQ (1000)
/* Lowest priority. As defined by the upper nibble. */
/* LSB nibble is don't care. */
#define SYSTICK_PRIO (0xF0)

/******************************************************************************
 * GPIO Configuration.
******************************************************************************/
#define GPIO_PORT_LED1 (GPIOA)
#define GPIO_PIN_LED1 (GPIO5)

/******************************************************************************
 * PWM Configuration.
******************************************************************************/
/* PWM definition / macros. */
#define PWM_PORT_X (GPIOA)
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

/******************************************************************************
 * ADC Configuration.
******************************************************************************/
typedef struct adc_conf_tag {
  uint32_t mod;
  uint8_t ch;
  uint16_t pin; 
  uint32_t port;
} adc_conf_t;

#define ADC_MOD_VBUS (ADC1)
#define ADC_CH_VBUS (2u)
#define ADC_GPIO_VBUS (GPIO1)
#define ADC_PORT_VBUS (GPIOA)

#define ADC_MOD_TEMP_SENS (ADC2) /* ADC12? */
#define ADC_CH_TEMP_SENS (14u)
#define ADC_GPIO_TEMP_SENS (GPIO2)
#define ADC_PORT_TEMP_SENS (GPIOC)

#define ADC_MOD_POT (ADC3)
#define ADC_CH_POT (1u)
#define ADC_PORT_POT (GPIOB)
#define ADC_GPIO_POT (GPIO1)

#define ADC_MOD_PHA_IFBK (ADC1)
#define ADC_CH_PHA_IFBK (1u)
#define ADC_GPIO_PHA_IFBK (GPIO0)
#define ADC_PORT_PHA_IFBK (GPIOA)

#define ADC_MOD_PHB_IFBK (ADC1)
#define ADC_CH_PHB_IFBK (7u)
#define ADC_GPIO_PHB_IFBK (GPIO1)
#define ADC_PORT_PHB_IFBK (GPIOC)

#define ADC_MOD_PHC_IFBK (ADC1)
#define ADC_CH_PHC_IFBK (6u)
#define ADC_GPIO_PHC_IFBK (GPIO0)
#define ADC_PORT_PHC_IFBK (GPIOC)

static void hw_init(void);
static void gpio_init(void);
static void clock_init(void);
static void pwm_init(void);
static void adc_init(void);
static void adc_inj_conv_cfg(uint32_t adc, uint8_t length, uint8_t channel[], uint32_t trigger, uint32_t polarity);

int main(void) {
  volatile uint16_t tmp_adc = 0u;
  hw_init();
  while(true) {
    while(!adc_eos(ADC1));
    tmp_adc = adc_read_injected(ADC1, 1);
  }
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
  adc_init();
  cm_enable_interrupts();
}

static void gpio_init(void) {
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);

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

  /* ADC input configuration. */
  gpio_mode_setup(
    GPIOA,
    GPIO_MODE_ANALOG,
    GPIO_PUPD_NONE,
    (ADC_GPIO_PHA_IFBK | ADC_GPIO_VBUS)
  );

  gpio_mode_setup(
    GPIOB,
    GPIO_MODE_ANALOG,
    GPIO_PUPD_NONE,
    (ADC_GPIO_POT)
  );

  gpio_mode_setup(
    GPIOC,
    GPIO_MODE_ANALOG,
    GPIO_PUPD_NONE,
    (ADC_GPIO_PHB_IFBK | ADC_GPIO_PHC_IFBK | ADC_GPIO_TEMP_SENS)
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

  /* Timer used to generate ADC sync. pulse. */
  timer_set_oc_slow_mode(TIM1, TIM_OC4);
  timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM2);
  timer_set_oc_value(TIM1, TIM_OC4, (PWM_TMR_PRD - 1));

  /* Master out configuration. */
  timer_enable_oc_output(TIM1, TIM_OC1);
  timer_enable_oc_output(TIM1, TIM_OC2);
  timer_enable_oc_output(TIM1, TIM_OC3);
  timer_enable_oc_output(TIM1, TIM_OC4);

  /* Generate TRGO event. */
  /* timer_set_master_mode(TIM1, TIM_CR2_MMS_COMPARE_OC4REF); */
  timer_set_master_mode(TIM1, TIM_CR2_MMS_UPDATE);

  /* Need to enable main output even if break / deadtime features are not used. */
  timer_enable_break_main_output(TIM1);
  timer_enable_counter(TIM1);
}

static void adc_init(void) {

  uint8_t adc_reg_conv_ch_list[] = {
    ADC_CH_POT,
  };

  uint8_t adc_inj_conv_ch_list[] = {
    ADC_CH_PHA_IFBK,
    ADC_CH_PHB_IFBK,
    ADC_CH_PHC_IFBK,
  };

  rcc_periph_clock_enable(RCC_ADC12);
  rcc_periph_clock_enable(RCC_ADC34);

  /* /1* Setup regular conversions. *1/ */
  /* adc_power_off(ADC3); */
  /* adc_set_clk_prescale(ADC3, ADC_CCR_CKMODE_DIV2); */
  /* adc_set_single_conversion_mode(ADC3); */
  /* /1* Equivalent to Software Start trigger. *1/ */
  /* adc_disable_external_trigger_regular(ADC3); */
  /* adc_set_right_aligned(ADC3); */
  /* adc_set_sample_time_on_all_channels(ADC3, ADC_SMPR_SMP_19DOT5CYC); */
  /* adc_set_regular_sequence(ADC3, sizeof(adc_reg_conv_ch_list), adc_reg_conv_ch_list); */
  /* adc_set_resolution(ADC3, ADC_CFGR1_RES_12_BIT); */
  /* adc_calibrate(ADC3); */
  /* adc_power_on(ADC3); */

  /* Setup injected conversions for motor phases. */
  adc_power_off(ADC1);
  adc_set_clk_prescale(ADC1, ADC_CCR_CKMODE_DIV2);
  adc_set_single_conversion_mode(ADC1);
  adc_set_right_aligned(ADC1);
  adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_19DOT5CYC);
  adc_enable_eos_interrupt_injected(ADC1);
  adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
  adc_disable_discontinuous_mode_regular(ADC1);
  adc_disable_discontinuous_mode_injected(ADC1);

  nvic_set_priority(NVIC_ADC1_2_IRQ, 0);
  nvic_enable_irq(NVIC_ADC1_2_IRQ);

  adc_disable_automatic_injected_group_conversion(ADC1);
  adc_disable_external_trigger_regular(ADC1);

  adc_calibrate(ADC1);
  adc_power_on(ADC1);

  /* adc_enable_external_trigger_injected( */
  /*     ADC1, */
  /*     ADC_JSQR_JEXTSEL_EVENT_0, */
  /*     ADC_JSQR_JEXTEN_RISING_EDGE */
  /* ); */
  /* adc_set_injected_sequence(ADC1, sizeof(adc_inj_conv_ch_list), adc_inj_conv_ch_list); */
  adc_inj_conv_cfg(ADC1, sizeof(adc_inj_conv_ch_list), adc_inj_conv_ch_list,
    ADC_JSQR_JEXTSEL_EVENT_0, ADC_JSQR_JEXTEN_RISING_EDGE);

  adc_start_conversion_injected(ADC1);
}

static void adc_inj_conv_cfg(uint32_t adc, uint8_t length, uint8_t channel[], uint32_t trigger, uint32_t polarity)
{
  uint32_t reg32 = 0;
  uint8_t i = 0;

  /* Maximum sequence length is 4 channels. Minimum sequence is 1.*/
  if ((length - 1) > 3) {
    return;
  }

  for (i = 0; i < length; i++) {
    reg32 |= ADC_JSQR_JSQ_VAL(4 - i, channel[length - i - 1]);
  }

  reg32 |= ADC_JSQR_JL_VAL(length);

  /* Do not write just yet. */
  /* ADC_JSQR(adc) = reg32; */
  reg32 &= ~(ADC_JSQR_JEXTSEL_MASK | ADC_JSQR_JEXTEN_MASK);
  reg32 |= (trigger | polarity);
  ADC_JSQR(adc) = reg32;

}

void adc1_2_isr(void)
{
  ADC_ISR(ADC1) &= ~((uint32_t)ADC_ISR_EOS | (uint32_t)ADC_ISR_EOC);
  ADC_ISR(ADC2) &= ~((uint32_t)ADC_ISR_EOS | (uint32_t)ADC_ISR_EOC);
}
