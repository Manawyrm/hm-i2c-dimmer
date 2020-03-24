#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#define BASE_PERIOD_LEN 100
#define PWM_CHANNEL_COUNT 2

typedef struct
{
	uint32_t port;
	uint16_t pin;
	uint8_t value;
} pwm_chan_t;

pwm_chan_t pwm_channels[PWM_CHANNEL_COUNT] = {
	{GPIOC, GPIO13, 0},
	{GPIOA, GPIO6, 0},
};

static void clock_setup(void)
{
	/* Set STM32 to 72 MHz. */
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIO A-C clock. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
}

static void gpio_setup(void)
{
	for (int i = 0; i < PWM_CHANNEL_COUNT; ++i)
	{
		/* Set PWM output GPIO to 'output push-pull'. */
		gpio_set_mode(pwm_channels[i].port, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, pwm_channels[i].pin);
	}
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
	 */
	timer_set_prescaler(TIM2, ((rcc_apb1_frequency / BASE_PERIOD_LEN) / 25000));

	/* Disable preload. */
	timer_disable_preload(TIM2);
	timer_continuous_mode(TIM2);

	/* count full range, as we'll update compare value continuously */
	timer_set_period(TIM2, BASE_PERIOD_LEN);

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

		timer_set_period(TIM2, BASE_PERIOD_LEN << pwm_bit);

		for (int i = 0; i < PWM_CHANNEL_COUNT; ++i) {
			if (!(pwm_channels[i].value & (1 << pwm_bit))) {
				gpio_set(pwm_channels[i].port, pwm_channels[i].pin);
			} else {
				gpio_clear(pwm_channels[i].port, pwm_channels[i].pin);
			}
		}		
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
		
		for (int i = 0; i < PWM_CHANNEL_COUNT; ++i)
		{
			pwm_channels[i].value++;
		}
	}

	return 0;
}
