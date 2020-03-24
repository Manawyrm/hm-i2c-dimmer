#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/i2c.h>

#define BASE_PERIOD_LEN 100
#define PWM_CHANNEL_COUNT 2

#define I2C_ADDRESS 0x32

typedef enum
{
	I2C_START, 
	I2C_REGISTER_SET,
	I2C_STOP,
} i2c_state_t;

i2c_state_t i2c_state = I2C_STOP;
uint8_t i2c_register = 0x00;

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

static void i2c_slave_init(uint8_t slave_address)
{
	nvic_enable_irq(NVIC_I2C1_EV_IRQ);

	// configure i2c pins
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
			 GPIO_I2C1_SDA); //PB7
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
			 GPIO_I2C1_SCL); //PB6

	i2c_reset(I2C1);	
	i2c_peripheral_disable(I2C1);

	i2c_set_speed(I2C1, i2c_speed_sm_100k, I2C_CR2_FREQ_36MHZ);
	i2c_set_own_7bit_slave_address(I2C1, slave_address);
	i2c_enable_interrupt(I2C1, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
	i2c_peripheral_enable(I2C1);

	// slave needs to acknowledge on receiving bytes
	// set it after enabling Peripheral i.e. PE = 1
	i2c_enable_ack(I2C1);
}

static void clock_setup(void)
{
	// Set STM32 to 72 MHz.
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	// Enable GPIO A-C clock.
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	// Enable I2C clock.
	rcc_periph_clock_enable(RCC_I2C1);
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

void i2c1_ev_isr(void)
{
	uint32_t sr1, sr2;

	sr1 = I2C_SR1(I2C1);

	// Address matched (Slave)
	if (sr1 & I2C_SR1_ADDR)
	{
		// Clear the ADDR sequence by reading SR2.
		i2c_state = I2C_START; 
		sr2 = I2C_SR2(I2C1);
		(void) sr2;
	}
	// Receive buffer not empty
	else if (sr1 & I2C_SR1_RxNE)
	{
		// Read register address
		if (i2c_state == I2C_START)
		{
			uint8_t tmp = i2c_get_data(I2C1);
			if (tmp < PWM_CHANNEL_COUNT)
			{
				i2c_register = tmp; 
				i2c_state = I2C_REGISTER_SET;
			}
			else
			{
				i2c_state = I2C_STOP;
			}
		}
		else if (i2c_state == I2C_REGISTER_SET)
		{
			pwm_channels[i2c_register].value = i2c_get_data(I2C1);
		} 
		else
		{
			// discard data
			i2c_get_data(I2C1);
		}
	}
	// Transmit buffer empty & Data byte transfer not finished
	else if ((sr1 & I2C_SR1_TxE) && !(sr1 & I2C_SR1_BTF))
	{
		if (i2c_state == I2C_START)
		{
			i2c_send_data(I2C1, pwm_channels[i2c_register].value);
		}
	}
	// done by master by sending STOP
	//this event happens when slave is in Recv mode at the end of communication
	else if (sr1 & I2C_SR1_STOPF)
	{
		i2c_peripheral_enable(I2C1);
		i2c_state = I2C_STOP;
	}
	//this event happens when slave is in transmit mode at the end of communication
	else if (sr1 & I2C_SR1_AF)
	{
		I2C_SR1(I2C1) &= ~(I2C_SR1_AF);
	}
}

int main(void)
{
	clock_setup();
	gpio_setup();
	timer_setup();
	i2c_slave_init(I2C_ADDRESS);

	while (1) {
		for (int i = 0; i < 1000000; i++)	/* Wait a bit. */
			__asm__("nop");
		
		/*for (int i = 0; i < PWM_CHANNEL_COUNT; ++i)
		{
			pwm_channels[i].value++;
		}*/
	}

	return 0;
}
