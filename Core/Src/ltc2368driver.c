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

	sampling_conf->tim_master->Instance->ARR = period-1;

	/* Set the Prescaler value */
	sampling_conf->tim_master->Instance->PSC = prescaler-1;
	/* Generate an update event to reload the Prescaler
	 and the repetition counter (only for advanced timer) value immediately */
	sampling_conf->tim_master->Instance->EGR = TIM_EGR_UG;
	/* Check if the update flag is set after the Update Generation, if so clear the UIF flag */
	if (HAL_IS_BIT_SET(sampling_conf->tim_master->Instance->SR, TIM_FLAG_UPDATE))
	{
	/* Clear the update flag */
		CLEAR_BIT(sampling_conf->tim_master->Instance->SR, TIM_FLAG_UPDATE);
	}
	sampling_conf->freq = frequency;
//	__HAL_TIM_ENABLE(sampling_conf->tim_master); //
	return LTC2368_OK;
}

LTC2368_StatusTypeDef LTC2368_SelectSource(LTC2368_SamplingClock *sampling_conf, uint32_t source){
	sampling_conf->ref_freq = source;
	if (source != SYSTEM_FREQ)

		SET_BIT(sampling_conf->tim_master->Instance->SMCR, TIM_SMCR_ECE|TIM_SMCR_ETP);
	else
		CLEAR_BIT(sampling_conf->tim_master->Instance->SMCR, TIM_SMCR_ECE);
	return LTC2368_OK;
}

LTC2368_StatusTypeDef LTC2368_StartSampling(LTC2368_SamplingClock *sampling_conf){
	return HAL_TIM_OC_Start(sampling_conf->tim_master, sampling_conf->tim_master_ch);
}

LTC2368_StatusTypeDef LTC2368_StopSampling(LTC2368_SamplingClock *sampling_conf){
	return HAL_TIM_OC_Stop(sampling_conf->tim_master, sampling_conf->tim_master_ch);
}



