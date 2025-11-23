/*
 * ltc2368driver.h
 *
 *  Created on: Aug 23, 2025
 *      Author: Celelele
 */

#ifndef INC_LTC2368DRIVER_H_
#define INC_LTC2368DRIVER_H_

#include "main.h"

#define LTC2368_MAX_MEMORY 16383 //2<<18 = 524288
#define SYSTEM_FREQ 64000000
#define REF_FREQ 10000000
#define MAX_SAMPLING_FREQ 1000000
#define MAX_SAMPLES_REQUESTED BUFFER_SIZE/4

typedef struct {
  uint32_t CTR1;
  uint32_t CTR2;
  uint32_t CBR1;
  uint32_t CSAR;
  uint32_t CDAR;
  uint32_t CLLR;
  uint32_t CTR3;
  uint32_t CBR2;
//  uint32_t RESERVED[8];
} DMA_Node;

typedef enum
{
	LTC2368_OK    	= 0x00U,
	LTC2368_ERROR	= 0x01U,
	LTC2368_BUSY    = 0x02U,
	LTC2368_TIMEOUT = 0x03U,
	LTC2368_DONE 	= 0x04U
} LTC2368_StatusTypeDef;

typedef enum
{
	INTR=0,
	EXTR
} LTC2368_SamplingMode;

typedef struct GPIO_Assignment {
	GPIO_TypeDef *port;
	uint16_t pin;
} GPIO_Assignment;

typedef struct LTC2368_Handler {
	uint8_t device_id;
	//GPIO_Assignment *cnv;
	GPIO_Assignment *busy_pin;
	uint32_t *buffer;
	uint32_t *buf_ptr;
	uint32_t *samples_requested;
	uint8_t *ready_to_disp;
} LTC2368_Handler;

typedef struct LTC2368_ClockHandler {
	TIM_TypeDef *instance; //drives cnv signal
	uint32_t ch;
	uint32_t ch_itr; //timer channel itr ch1-1, ch2-2, ch3-4, etc.
	bool is_advanced;
} LTC2368_ClockHandler;

typedef struct LTC2368_SamplingClock {
	GPIO_Assignment cnv_pin;
	uint32_t freq;
	uint32_t ref_freq;
	LTC2368_ClockHandler tim_master; //defines sampling frequency
	LTC2368_ClockHandler tim_slave; //drives cnv signal
	LTC2368_ClockHandler tim_delay; //controls communication timer
	LTC2368_ClockHandler tim_comm; //drives communication with adc
} LTC2368_SamplingClock;

LTC2368_StatusTypeDef LTC2368_Init(LTC2368_SamplingClock *sampling_clock, TIM_TypeDef *tim_master, uint32_t tim_master_ch, TIM_TypeDef *tim_slave, uint32_t tim_slave_ch, TIM_TypeDef *tim_delay, uint32_t tim_delay_ch, TIM_TypeDef *tim_comm, uint32_t tim_comm_ch);
LTC2368_StatusTypeDef LTC2368_Read(LTC2368_Handler *ltc2368_dev);
LTC2368_StatusTypeDef LTC2368_Convert(LTC2368_Handler *ltc2368_dev);
LTC2368_StatusTypeDef LTC2368_Unclock(LTC2368_Handler *ltc2368_dev);
LTC2368_StatusTypeDef LTC2368_Lock(LTC2368_Handler *ltc2368_dev);
LTC2368_StatusTypeDef LTC2368_ConfigSampling(LTC2368_SamplingClock *sampling_conf, uint32_t frequency);
LTC2368_StatusTypeDef LTC2368_SelectSource(LTC2368_SamplingClock *sampling_conf, uint32_t source);
LTC2368_StatusTypeDef LTC2368_ArmTimers(LTC2368_SamplingClock *sampling_conf);
LTC2368_StatusTypeDef LTC2368_EnableTimer(TIM_TypeDef *tim, uint32_t channel);
LTC2368_StatusTypeDef LTC2368_DisableTimer(TIM_TypeDef *tim, uint32_t channel);
LTC2368_StatusTypeDef LTC2368_EnableTimer_IT(TIM_TypeDef *tim, uint32_t channel, uint32_t *channel_itr);
LTC2368_StatusTypeDef LTC2368_DisableTimer_IT(TIM_TypeDef *tim, uint32_t channel, uint32_t *channel_itr);
LTC2368_StatusTypeDef LTC2368_StartSampling(LTC2368_SamplingClock *sampling_conf);
LTC2368_StatusTypeDef LTC2368_StopSampling(LTC2368_SamplingClock *sampling_conf);
void LTC2368_SlaveIrqHandling(LTC2368_ClockHandler *tim_delay, LTC2368_ClockHandler *tim_comm, volatile uint32_t *counter);
LTC2368_StatusTypeDef LTC2368_DisplaySamples(LTC2368_Handler *ltc2368_dev, uint16_t requested_samples, char *text_buf);

#endif /* INC_LTC2368DRIVER_H_ */
