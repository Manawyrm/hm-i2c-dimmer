#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#define BASE_PERIOD_LEN 100

volatile uint8_t pwm_value = 254;

static void clock_setup(void)
{
	/* Set STM32 to 72 MHz. */
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);
}

static void gpio_setup(void)
{
	/* Set GPIO12 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);
}

static void timer_setup(void)
{
	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);

	/* Enable TIM2 interrupt. */
	nvic_enable_irq(NVIC_TIM2_IRQ);

	/* Reset TIM2 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM2);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/*
	 * Please take note that the clock source for STM32 timers
	 * might not be the raw APB1/APB2 clocks.  In various conditions they
	 * are doubled.  See the Reference Manual for full details!
	 * In our case, TIM2 on APB1 is running at double frequency, so this
	 * sets the prescaler to have the timer run at 5kHz
	 */
	timer_set_prescaler(TIM2, ((rcc_apb1_frequency / BASE_PERIOD_LEN) / 25000));

	/* Disable preload. */
	timer_disable_preload(TIM2);
	timer_continuous_mode(TIM2);

	/* count full range, as we'll update compare value continuously */
	timer_set_period(TIM2, BASE_PERIOD_LEN);

	/* Set the initual output compare value for OC1. */
	//timer_set_oc_value(TIM2, TIM_OC1, frequency_sequence[frequency_sel++]);

	/* Counter enable. */
	timer_enable_counter(TIM2);

	/* Enable Channel 1 compare interrupt to recalculate compare values */
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}

volatile uint8_t pwm_bit = 0;

void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_CC1IF))
	{
		/* Clear compare interrupt flag. */
		timer_clear_flag(TIM2, TIM_SR_CC1IF);

		pwm_bit++;
		pwm_bit %= 8;
		if (!pwm_bit)
			gpio_toggle(GPIOA, GPIO6);


		timer_set_period(TIM2, BASE_PERIOD_LEN << pwm_bit);

		if (pwm_value & (1 << pwm_bit))
			gpio_clear(GPIOC, GPIO13);
		else
			gpio_set(GPIOC, GPIO13);
	}
}


int main(void)
{
	clock_setup();
	gpio_setup();
	timer_setup();

	while (1) {
		for (int i = 0; i < 1000000; i++)	/* Wait a bit. */
			__asm__("nop");
		pwm_value++;
	}

	return 0;
}
