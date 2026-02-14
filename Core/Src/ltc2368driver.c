/*
 * ltc2368driver.c
 *
 *  Created on: Aug 23, 2025
 *      Author: Celelele
 */

#include "ltc2368driver.h"

/*
 * LTC2368_CountFreq - Calculate timer output frequency from base frequency
 * @param tim: Pointer to timer peripheral
 * @param base: Base input frequency in Hz
 * @return Calculated output frequency in Hz
 * 
 * Calculates the timer output frequency based on prescaler (PSC) and auto-reload (ARR) values.
 * Formula: output_freq = base_freq / ((PSC+1) * (ARR+1))
 * Used for determining actual sampling and reading frequencies.
 */
static inline uint32_t LTC2368_CountFreq(TIM_TypeDef *tim, uint32_t base){
	uint32_t base_freq = (uint32_t)(base/(tim->PSC+1));
	uint32_t output_freq = (uint32_t)(base_freq/(tim->ARR+1));
	return output_freq;
}

/*
 * LTC2368_Init - Initialize LTC2368 ADC clock configuration structure
 * @param sampling_clock: Pointer to sampling clock structure to initialize
 * @param tim_master: Pointer to master timer (defines sampling frequency)
 * @param tim_master_ch: Master timer channel
 * @param tim_slave: Pointer to slave timer (drives CNV signal)
 * @param tim_slave_ch: Slave timer channel
 * @param tim_delay: Pointer to delay timer (controls communication timing)
 * @param tim_delay_ch: Delay timer channel
 * @param tim_comm: Pointer to communication timer (drives ADC communication)
 * @param tim_comm_ch: Communication timer channel
 * @return LTC2368_OK on success
 * 
 * Initializes the LTC2368 sampling clock structure with timer instances and channels.
 * Detects if timers are advanced timers, calculates initial sampling and reading frequencies,
 * and sets the default reference frequency to SYSTEM_FREQ. This function should be called
 * before configuring sampling parameters.
 */
LTC2368_StatusTypeDef LTC2368_Init(LTC2368_SamplingClock *sampling_clock, TIM_TypeDef *tim_master, uint32_t tim_master_ch, TIM_TypeDef *tim_slave, uint32_t tim_slave_ch, TIM_TypeDef *tim_delay, uint32_t tim_delay_ch, TIM_TypeDef *tim_comm, uint32_t tim_comm_ch){
	sampling_clock->tim_master.instance = tim_master;
	sampling_clock->tim_master.ch = tim_master_ch;
	sampling_clock->tim_master.is_advanced = (sampling_clock->tim_master.instance)?true:false;
	sampling_clock->tim_slave.instance = tim_slave;
	sampling_clock->tim_slave.ch = tim_slave_ch;
	sampling_clock->tim_slave.is_advanced = (sampling_clock->tim_slave.instance)?true:false;
	sampling_clock->tim_delay.instance = tim_delay;
	sampling_clock->tim_delay.ch = tim_delay_ch;
	sampling_clock->tim_delay.is_advanced = (sampling_clock->tim_delay.instance)?true:false;
	sampling_clock->tim_comm.instance = tim_comm;
	sampling_clock->tim_comm.ch = tim_comm_ch;
	sampling_clock->tim_comm.is_advanced = (sampling_clock->tim_comm.instance)?true:false;
	sampling_clock->ref_freq = SYSTEM_FREQ;
	sampling_clock->samp_freq = LTC2368_CountFreq(tim_master, sampling_clock->ref_freq); //2000
	sampling_clock->read_freq = LTC2368_CountFreq(tim_comm, SYSTEM_FREQ);
	return LTC2368_OK;
}

/*
 * LTC2368_ConfigSampling - Configure ADC sampling frequency
 * @param sampling_conf: Pointer to sampling clock configuration
 * @param frequency: Desired sampling frequency in Hz
 * @return LTC2368_OK on success
 * 
 * Configures the master timer to generate the specified sampling frequency.
 * Calculates appropriate prescaler and period values based on the reference frequency.
 * For internal clock (SYSTEM_FREQ), uses both prescaler and period adjustment.
 * For external clock, adjusts period only. Updates the sampling frequency in the
 * configuration structure after calculation.
 */
LTC2368_StatusTypeDef LTC2368_ConfigSampling(LTC2368_SamplingClock *sampling_conf, uint32_t frequency){

//	__HAL_TIM_DISABLE(sampling_conf->tim_master); //

	uint32_t period = 1000;
	uint32_t prescaler = 1;
	if (sampling_conf->ref_freq == SYSTEM_FREQ)
	{
		while((prescaler*period*frequency)<sampling_conf->ref_freq){
			prescaler++;
		}
		while((prescaler*period*frequency)>sampling_conf->ref_freq){
			period--;
		}
	}
	else
	{
		period = 1;
		while((period*frequency)<sampling_conf->ref_freq){
			period++;
		}
	}

	sampling_conf->tim_master.instance->ARR = period-1;

	/* Set the Prescaler value */
	sampling_conf->tim_master.instance->PSC = prescaler-1;
	/* Generate an update event to reload the Prescaler
	 and the repetition counter (only for advanced timer) value immediately */
	sampling_conf->tim_master.instance->EGR = TIM_EGR_UG;
	/* Check if the update flag is set after the Update Generation, if so clear the UIF flag */
	if (HAL_IS_BIT_SET(sampling_conf->tim_master.instance->SR, TIM_FLAG_UPDATE))
	{
	/* Clear the update flag */
		CLEAR_BIT(sampling_conf->tim_master.instance->SR, TIM_FLAG_UPDATE);
	}
	sampling_conf->samp_freq = LTC2368_CountFreq(sampling_conf->tim_master.instance, sampling_conf->ref_freq);
//	__HAL_TIM_ENABLE(sampling_conf->tim_master); //
	return LTC2368_OK;
}

/*
 * LTC2368_ConfigReading - Configure ADC reading clock prescaler
 * @param sampling_conf: Pointer to sampling clock configuration
 * @param prescaler: Prescaler value for reading clock (must be <= PSC_MAX)
 * @return LTC2368_OK on success, LTC2368_ERROR if prescaler exceeds maximum
 * 
 * Sets the prescaler for the communication timer that drives ADC read operations.
 * Updates the reading frequency in the configuration structure. The reading clock
 * controls the speed at which data is read from the ADC via PSSI interface.
 */
LTC2368_StatusTypeDef LTC2368_ConfigReading(LTC2368_SamplingClock *sampling_conf, uint32_t prescaler){
	if (prescaler > PSC_MAX) return LTC2368_ERROR;
	sampling_conf->tim_comm.instance->PSC = prescaler;
	sampling_conf->read_freq = LTC2368_CountFreq(sampling_conf->tim_comm.instance, SYSTEM_FREQ);
	return LTC2368_OK;
}

/*
 * LTC2368_AdjustPrescaler - Automatically select prescaler based on sampling frequency
 * @param sampling_conf: Pointer to sampling clock configuration
 * @param frequency: Current sampling frequency in Hz
 * @param prescaler: Output parameter for selected prescaler value
 * @return LTC2368_OK on success
 * 
 * Selects an appropriate prescaler value for the reading clock based on the
 * sampling frequency. Uses a lookup table to match frequency ranges to prescaler
 * values (3-80) to ensure reliable data communication timing.
 */
LTC2368_StatusTypeDef LTC2368_AdjustPrescaler(LTC2368_SamplingClock *sampling_conf, uint32_t frequency, uint32_t *prescaler){
	if (frequency > 550000) *prescaler = 3;
	else if (frequency > 500000) *prescaler = 4;
	else if (frequency > 450000) *prescaler = 5;
	else if (frequency > 400000) *prescaler = 6;
	else if (frequency > 350000) *prescaler = 7;
	else if (frequency > 300000) *prescaler = 8;
	else if (frequency > 250000) *prescaler = 9;
	else if (frequency > 200000) *prescaler = 10;
	else if (frequency > 100000) *prescaler = 20;
	else if (frequency > 50000) *prescaler = 40;
	else *prescaler = 80;
	return LTC2368_OK;
}

/*
 * LTC2368_ArmAdvancedTimer - Enable main output for advanced timers
 * @param tim: Pointer to timer peripheral
 * @param is_advanced: True if timer is an advanced timer
 * @return none
 * 
 * Enables the main output enable (MOE) bit in the BDTR register for advanced timers.
 * This is required for advanced timers to output signals. Basic timers are not affected.
 */
static void inline LTC2368_ArmAdvancedTimer(TIM_TypeDef *tim, bool is_advanced){
	if (is_advanced)
		tim->BDTR|=(TIM_BDTR_MOE);
}

/*
 * LTC2368_ArmTimers - Enable and configure all ADC timing timers
 * @param sampling_conf: Pointer to sampling clock configuration
 * @return LTC2368_OK on success
 * 
 * Arms all timers used for ADC operation:
 * - Slave timer: Enabled with interrupt for CNV signal generation
 * - Communication timer: Enabled for ADC data read timing
 * - Master timer: Main output enabled (if advanced)
 * - Delay timer: Enabled for communication delay timing
 * This function should be called before starting sampling.
 */
LTC2368_StatusTypeDef LTC2368_ArmTimers(LTC2368_SamplingClock *sampling_conf){
	LTC2368_EnableTimer_IT(sampling_conf->tim_slave.instance, sampling_conf->tim_slave.ch, &sampling_conf->tim_slave.ch_itr);
	LTC2368_EnableTimer(sampling_conf->tim_slave.instance, sampling_conf->tim_slave.ch);
	LTC2368_ArmAdvancedTimer(sampling_conf->tim_slave.instance, sampling_conf->tim_slave.is_advanced);
	LTC2368_EnableTimer(sampling_conf->tim_comm.instance, sampling_conf->tim_comm.ch);   // arming timer, waiting for TRIG
	LTC2368_ArmAdvancedTimer(sampling_conf->tim_comm.instance, sampling_conf->tim_comm.is_advanced);
	LTC2368_ArmAdvancedTimer(sampling_conf->tim_master.instance, sampling_conf->tim_master.is_advanced);
	LTC2368_EnableTimer(sampling_conf->tim_delay.instance, sampling_conf->tim_delay.ch);   // arming timer, waiting for TRIG
	LTC2368_ArmAdvancedTimer(sampling_conf->tim_delay.instance, sampling_conf->tim_delay.is_advanced);
	return LTC2368_OK;
}

/*
 * LTC2368_SelectSource - Select clock source for ADC sampling
 * @param sampling_conf: Pointer to sampling clock configuration
 * @param source: Clock source frequency in Hz (SYSTEM_FREQ for internal, REF_FREQ for external, or custom)
 * @return LTC2368_OK on success
 * 
 * Configures the master timer to use internal or external clock source:
 * - SYSTEM_FREQ: Internal system clock (clears external clock enable)
 * - Other values: External clock (enables external clock mode with edge detection)
 * Updates the reference frequency in the configuration structure.
 */
LTC2368_StatusTypeDef LTC2368_SelectSource(LTC2368_SamplingClock *sampling_conf, uint32_t source){
	sampling_conf->ref_freq = source;
	if (source != SYSTEM_FREQ)

		SET_BIT(sampling_conf->tim_master.instance->SMCR, TIM_SMCR_ECE|TIM_SMCR_ETP);
	else
		CLEAR_BIT(sampling_conf->tim_master.instance->SMCR, TIM_SMCR_ECE);
	return LTC2368_OK;
}

/*
 * LTC2368_EnableTimer - Enable a timer channel and start counting
 * @param tim: Pointer to timer peripheral
 * @param channel: Timer channel to enable
 * @return LTC2368_OK on success
 * 
 * Resets the timer counter to 0, enables the specified channel output,
 * and starts the timer by setting the counter enable (CEN) bit.
 */
LTC2368_StatusTypeDef LTC2368_EnableTimer(TIM_TypeDef *tim, uint32_t channel){
	tim->CNT = 0;
	//enabling channel
	tim->CCER |= TIM_CCER_CC1E << (channel & 0x1FU);
	//starting timer
	tim->CR1 |= TIM_CR1_CEN;
	return LTC2368_OK;
}

/*
 * LTC2368_DisableTimer - Disable a timer channel and stop counting
 * @param tim: Pointer to timer peripheral
 * @param channel: Timer channel to disable
 * @return LTC2368_OK on success
 * 
 * Disables the specified channel output and stops the timer by clearing
 * the counter enable (CEN) bit.
 */
LTC2368_StatusTypeDef LTC2368_DisableTimer(TIM_TypeDef *tim, uint32_t channel){
	//enabling channel
	tim->CCER &= ~(TIM_CCER_CC1E << (channel & 0x1FU));
	//starting timer
	tim->CR1 &= ~TIM_CR1_CEN;
	return LTC2368_OK;
}

LTC2368_StatusTypeDef LTC2368_ResetTimer(TIM_TypeDef *tim){
	//restart counter
	tim->CNT = 0;
	return LTC2368_OK;
}

/*
 * LTC2368_EnableTimer_IT - Enable timer interrupt for a specific channel
 * @param tim: Pointer to timer peripheral
 * @param channel: Timer channel (1-4)
 * @param channel_itr: Output parameter for interrupt flag (TIM_IT_CC1-CC4)
 * @return LTC2368_OK on success, LTC2368_ERROR for invalid channel
 * 
 * Enables the capture/compare interrupt for the specified timer channel
 * and stores the corresponding interrupt flag in channel_itr. Valid channels
 * are 1-4, corresponding to TIM_CHANNEL_1 through TIM_CHANNEL_4.
 */
LTC2368_StatusTypeDef LTC2368_EnableTimer_IT(TIM_TypeDef *tim, uint32_t channel, uint32_t *channel_itr){
	LTC2368_StatusTypeDef status = LTC2368_OK;
	switch (channel)
	{
		case TIM_CHANNEL_1:
		{
			/* Assign channel itr to handler */
			*channel_itr = TIM_IT_CC1;
			/* Enable the TIM Capture/Compare 1 interrupt */
			tim->DIER |= TIM_IT_CC1;
			break;
		}

		case TIM_CHANNEL_2:
		{
			/* Assign channel itr to handler */
			*channel_itr = TIM_IT_CC2;
			/* Enable the TIM Capture/Compare 2 interrupt */
			tim->DIER |= TIM_IT_CC2;
			break;
		}

		case TIM_CHANNEL_3:
		{
			/* Assign channel itr to handler */
			*channel_itr = TIM_IT_CC3;
			/* Enable the TIM Capture/Compare 3 interrupt */
			tim->DIER |= TIM_IT_CC3;
			break;
		}

		case TIM_CHANNEL_4:
		{
			/* Assign channel itr to handler */
			*channel_itr = TIM_IT_CC4;
			/* Enable the TIM Capture/Compare 4 interrupt */
			tim->DIER |= TIM_IT_CC4;
			break;
		}

		default:
			status = LTC2368_ERROR;
			break;
	}
	return status;
}

/*
 * LTC2368_DisableTimer_IT - Disable timer interrupt for a specific channel
 * @param tim: Pointer to timer peripheral
 * @param channel: Timer channel (1-4)
 * @param channel_itr: Interrupt flag (not used, kept for API consistency)
 * @return LTC2368_OK on success, LTC2368_ERROR for invalid channel
 * 
 * Disables the capture/compare interrupt for the specified timer channel.
 * Valid channels are 1-4, corresponding to TIM_CHANNEL_1 through TIM_CHANNEL_4.
 */
LTC2368_StatusTypeDef LTC2368_DisableTimer_IT(TIM_TypeDef *tim, uint32_t channel, uint32_t *channel_itr){
	LTC2368_StatusTypeDef status = LTC2368_OK;
	switch (channel)
	{
		case TIM_CHANNEL_1:
		{
			/* Enable the TIM Capture/Compare 1 interrupt */
			tim->DIER &= ~(TIM_IT_CC1);
			break;
		}

		case TIM_CHANNEL_2:
		{
			/* Enable the TIM Capture/Compare 2 interrupt */
			tim->DIER &= ~(TIM_IT_CC2);
			break;
		}

		case TIM_CHANNEL_3:
		{
			/* Enable the TIM Capture/Compare 3 interrupt */
			tim->DIER &= ~(TIM_IT_CC3);
			break;
		}

		case TIM_CHANNEL_4:
		{
			/* Enable the TIM Capture/Compare 4 interrupt */
			tim->DIER &= ~(TIM_IT_CC4);
			break;
		}

		default:
			status = LTC2368_ERROR;
			break;
	}
	return status;
}

/*
 * LTC2368_StartSampling - Start ADC sampling by enabling master timer
 * @param sampling_conf: Pointer to sampling clock configuration
 * @return LTC2368_OK on success
 * 
 * Starts the ADC sampling process by enabling the master timer. The master timer
 * generates periodic triggers that initiate ADC conversions. This function should
 * be called after all timers are armed and configured.
 */
LTC2368_StatusTypeDef LTC2368_StartSampling(LTC2368_SamplingClock *sampling_conf){
	return LTC2368_EnableTimer(sampling_conf->tim_master.instance, sampling_conf->tim_master.ch);
}

/*
 * LTC2368_StopSampling - Stop ADC sampling by disabling master timer
 * @param sampling_conf: Pointer to sampling clock configuration
 * @return LTC2368_OK on success
 * 
 * Stops the ADC sampling process by disabling the master timer. This halts
 * the generation of conversion triggers. Used to stop sampling operations.
 */
LTC2368_StatusTypeDef LTC2368_StopSampling(LTC2368_SamplingClock *sampling_conf){
	if (LTC2368_DisableTimer(sampling_conf->tim_master.instance, sampling_conf->tim_master.ch) != LTC2368_OK) return LTC2368_ERROR;
//	if (LTC2368_ResetTimer(sampling_conf->tim_comm.instance) != LTC2368_OK) return LTC2368_ERROR;
	return LTC2368_OK;
}

/*
 * LTC2368_SlaveIrqHandling - Handle slave timer interrupt for ADC timing
 * @param tim_delay: Pointer to delay timer handler (enabled by this interrupt)
 * @param tim_slave: Pointer to slave timer handler (source of interrupt)
 * @param counter: Pointer to sample counter (incremented on each interrupt)
 * @return none
 * 
 * Handles the slave timer interrupt which occurs when CNV signal is generated.
 * Clears the interrupt flag, enables the delay timer for communication timing,
 * and increments the sample counter. This function should be registered as the
 * interrupt handler for the slave timer.
 */
void LTC2368_SlaveIrqHandling(LTC2368_ClockHandler *tim_delay, LTC2368_ClockHandler *tim_slave, volatile uint32_t *counter){
	uint32_t itflag   = tim_slave->instance->SR;
	if ((itflag & (tim_slave->ch_itr)) == (tim_slave->ch_itr))
		tim_slave->instance->SR = ~(tim_slave->ch_itr); //__HAL_TIM_CLEAR_FLAG

	/*communication timer is enabled by master timer tim_delay*/
//	LTC2368_EnableTimer(g_adc_mgr->clock_handler.tim_comm, g_adc_mgr->clock_handler.tim_comm_ch);

	LTC2368_EnableTimer(tim_delay->instance, tim_delay->ch);
	(*counter)++;
}



