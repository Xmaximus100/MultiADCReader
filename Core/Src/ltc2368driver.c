/*
 * ltc2368driver.c
 *
 *  Created on: Aug 23, 2025
 *      Author: Celelele
 */

#include "ltc2368driver.h"


//LTC2368_StatusTypeDef LTC2368_Read(LTC2368_Handler *ltc2368_dev){
//	if (ltc2368_dev->buf_ptr == *ltc2368_dev->samples_requested) {
//		*ltc2368_dev->ready_to_disp |= 1<<ltc2368_dev->device_id;
//		return LTC2368_DONE;
//	}
//	LTC2368_StatusTypeDef status = HAL_SPI_Receive(ltc2368_dev->spi_assinged->spi_handler, (uint8_t*)&ltc2368_dev->buf[ltc2368_dev->buf_ptr], 1, HAL_MAX_DELAY);
//	ltc2368_dev->buf_ptr = (ltc2368_dev->buf_ptr+1)&(LTC2368_MAX_MEMORY-1);
//	return status;
//}

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
	sampling_clock->freq = 2000;
	return LTC2368_OK;
}

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
	sampling_conf->freq = frequency;
//	__HAL_TIM_ENABLE(sampling_conf->tim_master); //
	return LTC2368_OK;
}

static void inline LTC2368_ArmAdvancedTimer(TIM_TypeDef *tim, bool is_advanced){
	if (is_advanced)
		tim->BDTR|=(TIM_BDTR_MOE);
}

LTC2368_StatusTypeDef LTC2368_ArmTimers(LTC2368_SamplingClock *sampling_conf){
	LTC2368_EnableTimer_IT(sampling_conf->tim_slave.instance, sampling_conf->tim_slave.ch, &sampling_conf->tim_slave.ch_itr);
	LTC2368_ArmAdvancedTimer(sampling_conf->tim_slave.instance, sampling_conf->tim_slave.is_advanced);
	LTC2368_EnableTimer(sampling_conf->tim_comm.instance, sampling_conf->tim_comm.ch);   // arming timer, waiting for TRIG
	LTC2368_ArmAdvancedTimer(sampling_conf->tim_comm.instance, sampling_conf->tim_comm.is_advanced);
	LTC2368_ArmAdvancedTimer(sampling_conf->tim_master.instance, sampling_conf->tim_master.is_advanced);
	LTC2368_EnableTimer(sampling_conf->tim_delay.instance, sampling_conf->tim_delay.ch);   // arming timer, waiting for TRIG
	LTC2368_ArmAdvancedTimer(sampling_conf->tim_delay.instance, sampling_conf->tim_delay.is_advanced);
	return LTC2368_OK;
}

LTC2368_StatusTypeDef LTC2368_SelectSource(LTC2368_SamplingClock *sampling_conf, uint32_t source){
	sampling_conf->ref_freq = source;
	if (source != SYSTEM_FREQ)

		SET_BIT(sampling_conf->tim_master.instance->SMCR, TIM_SMCR_ECE|TIM_SMCR_ETP);
	else
		CLEAR_BIT(sampling_conf->tim_master.instance->SMCR, TIM_SMCR_ECE);
	return LTC2368_OK;
}

LTC2368_StatusTypeDef LTC2368_EnableTimer(TIM_TypeDef *tim, uint32_t channel){
	tim->CNT = 0;
	//enabling channel
	tim->CCER |= TIM_CCER_CC1E << (channel & 0x1FU);
	//starting timer
	tim->CR1 |= TIM_CR1_CEN;
	return LTC2368_OK;
}

LTC2368_StatusTypeDef LTC2368_DisableTimer(TIM_TypeDef *tim, uint32_t channel){
	//enabling channel
	tim->CCER &= ~(TIM_CCER_CC1E << (channel & 0x1FU));
	//starting timer
	tim->CR1 &= ~TIM_CR1_CEN;
	return LTC2368_OK;
}

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

LTC2368_StatusTypeDef LTC2368_StartSampling(LTC2368_SamplingClock *sampling_conf){
	return LTC2368_EnableTimer(sampling_conf->tim_master.instance, sampling_conf->tim_master.ch);
//	__HAL_TIM_ENABLE(&htim4); // CEN=1 → jeden okres (OPM)
//	return HAL_TIM_OC_Start(sampling_conf->tim_master, sampling_conf->tim_master_ch);
//	return LTC2368_OK;
}

LTC2368_StatusTypeDef LTC2368_StopSampling(LTC2368_SamplingClock *sampling_conf){
//	sampling_conf->tim_master->CNT = 0;
//	TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_DISABLE);
	return LTC2368_DisableTimer(sampling_conf->tim_master.instance, sampling_conf->tim_master.ch);
//	return LTC2368_OK;
//	__HAL_TIM_DISABLE(&htim4); // CEN=1 → jeden okres (OPM)
//	return HAL_TIM_OC_Stop(sampling_conf->tim_master, sampling_conf->tim_master_ch);
}

void LTC2368_SlaveIrqHandling(LTC2368_ClockHandler *tim_delay, LTC2368_ClockHandler *tim_slave, volatile uint32_t *counter){
	uint32_t itflag   = tim_slave->instance->SR;
	if ((itflag & (tim_slave->ch_itr)) == (tim_slave->ch_itr))
		tim_slave->instance->SR = ~(tim_slave->ch_itr); //__HAL_TIM_CLEAR_FLAG
	/*communication timer is enabled by master timer tim_delay*/
//	LTC2368_EnableTimer(g_adc_mgr->clock_handler.tim_comm, g_adc_mgr->clock_handler.tim_comm_ch);

	LTC2368_EnableTimer(tim_delay->instance, tim_delay->ch);
	(*counter)++;
}



