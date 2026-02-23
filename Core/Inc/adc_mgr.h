/*
 * adc_mgr.h
 *
 *  Created on: Aug 28, 2025
 *      Author: Celelele
 */

#ifndef INC_ADC_MGR_H_
#define INC_ADC_MGR_H_

#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "ringbuffer.h"
#include "at_parser.h"
#include "ltc2368driver.h"

#define MAX_DEVICES 8
#define SPI_AMOUNT 2
#define MAX_EXTI 16
#define REFRENCE_V	2500 	//reference voltage in mV
#define RESOLUTION_BITS	65536
#define MAX_BUFFER_SIZE 128

/*
 * ADC_MGR is responsible for handling multiple ltc2368, assigning proper SPI instances
 * and providing configuration and readings via USB CDC
*/

typedef struct {
	volatile uint32_t ready_mask;				//ready mask for quick adc data collection
	LTC2368_SamplingClock clock_handler;		//slave and master clock handlers with sampling frquency defined
	GPIO_Assignment busy_pins[MAX_DEVICES];  	//all exposed busy pins, if empty - NULL
	LTC2368_Handler ltc2368_devs[MAX_DEVICES];	//all connected devices, if empty - NULL
	uint32_t *common_buffer;
	uint16_t output_buf[MAX_DEVICES];
	uint8_t usb_stream[MAX_BUFFER_SIZE];
	__IO uint32_t common_ptr;
	uint8_t exti_to_dev[MAX_EXTI];				//pin to ID list
	uint8_t ndevs;								//connected devices
	bool ready_to_disp;							//ready to display status
	bool continuous;
	uint16_t ready_to_disp_mask;				//mask for ready to display check
	uint8_t format; 							//display format 0-raw, 1-readable
	uint32_t refresh_interval; 					//display periods
	uint32_t display_samples; 					//samples to display at once
	uint32_t samples_requested;					//samples requested to collected, common for each device
	uint32_t samples_collected;
	AT_CtxT display_func;						//function for displaying data
	bool state; 								//adc on/off
	bool done;
	DMA_Node *chx_lli[4];						//list of linked list pointers
	DMA_Channel_TypeDef* dma_handler;			//main dma handler
	__IO uint32_t *dma_flags_reg;
	uint32_t dma_tc_flag_mask;
	uint32_t dma_us_flag_mask;
	int32_t nodes_used;
} ADC_Handler;

enum {
	KEEP_BUF=0,
	RESET_BUF
};

extern ADC_Handler g_adc;
extern ADC_Handler * const g_adc_mgr;

bool ADC_Init(ADC_Handler *m, DMA_Node *chx_lli, DMA_Channel_TypeDef *dma_handler, uint32_t* buffer, const GPIO_Assignment busy_pins[], AT_WriteFunc func, void *user);
bool ADC_ManagerInit(ADC_Handler *m, uint8_t dev_amount, bool mode);
void ADC_Acquire(ADC_Handler *m);
bool ADC_StartSampling(void);
bool ADC_StopSampling(void);
void ADC_MarkReady(ADC_Handler *m);
uint8_t ADC_PinToIndex(uint16_t GPIO_Pin);
bool ADC_DisplayConfig(ADC_Handler *m);
bool ADC_DisplaySamples_Clear(ADC_Handler *m, bool reset_buf, uint32_t custom_samples_requested);
bool ADC_DisplaySamples_Raw(ADC_Handler *m, bool reset_buf, uint32_t custom_samples_requested);
bool ADC_BusyCheck(void);
void ADC_ChangeRequestedSamples(ADC_Handler *m, uint16_t new_request);
void ADC_TIM_IRQHandler(void);
void ADC_DMA_IRQHandler(void);

#endif /* INC_ADC_MGR_H_ */
