/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h5xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h5xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc_mgr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t collect_data = 0;
uint32_t user_code_error = 0;
uint8_t button_pressed = 0;

extern ADC_Handler * const g_adc_mgr;
extern __IO uint32_t PSSI_HAL_PSSI_ReceiveComplete_count;
extern uint32_t *node;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef handle_GPDMA1_Channel7;
extern TIM_HandleTypeDef htim2;
extern PCD_HandleTypeDef hpcd_USB_DRD_FS;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim3;
//extern uint16_t *node;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h5xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles GPDMA1 Channel 7 global interrupt.
  */
void GPDMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel7_IRQn 0 */

//	__HAL_TIM_ENABLE(&htim8); // CEN=1 → jeden okres (OPM)
//	HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);  // lub __HAL_TIM_ENABLE(&htim_master)

	DMA_Channel_TypeDef *ch = GPDMA1_Channel7_NS;

	uint32_t sr = ch->CSR;                                                                /* :contentReference[oaicite:14]{index=14} */

	if (sr & DMA_CSR_TCF) {
		LTC2368_StopSampling(&g_adc_mgr->clock_handler);/* Transfer complete */     /* :contentReference[oaicite:15]{index=15} */
		WRITE_REG(ch->CFCR, DMA_CFCR_TCF);                                                  /* clear TC */              /* :contentReference[oaicite:16]{index=16} */
		//	    /* pseudo-CIRC: odśwież BNDT i ponownie włącz kanał */
		//	    MODIFY_REG(ch->CBR1, DMA_CBR1_BNDT, (BUFFER_SIZE & DMA_CBR1_BNDT_Msk));            /* :contentReference[oaicite:17]{index=17} */
		//		TIM3->CCER &= ~TIM_CCER_CC1E;              // pin OFF
		//		__HAL_TIM_DISABLE(&htim3);
		//		TIM3->CCER |= TIM_CCER_CC1E;              // pin ON

//		WRITE_REG(ch->CLBAR, (uint32_t)g_adc_mgr->chx_lli);        /* LBA = baza 4KB = adres węzła */
//		WRITE_REG(ch->CLLR,  DMA_CLLR_ULL                 /* update link addr z węzła    */
//						   |  DMA_CLLR_USA                /* ZAŁADUJ SAR                 */
//						   |  DMA_CLLR_UDA                /* ZAŁADUJ DAR                 */
//						   |  DMA_CLLR_UB1                /* ZAŁADUJ BR1                 */
//						   |  DMA_CLLR_UT1                /* ZAŁADUJ TR1                 */
//						   |  DMA_CLLR_UT2);              /* ZAŁADUJ TR2                 */
//		SET_BIT(ch->CCR, DMA_CCR_EN);                                                       /* restart */               /* :contentReference[oaicite:18]{index=18} */
		PSSI_HAL_PSSI_ReceiveComplete_count++;
	}

	//	  if (sr & DMA_CSR_HTF)  WRITE_REG(ch->CFCR, DMA_CFCR_HTF);                             /* clear HT */              /* :contentReference[oaicite:19]{index=19}turn20file2 */
	//	  if (sr & DMA_CSR_DTEF) WRITE_REG(ch->CFCR, DMA_CFCR_DTEF);                            /* clear DTE */             /* :contentReference[oaicite:20]{index=20}turn20file2 */
	//	  if (sr & DMA_CSR_TOF)  WRITE_REG(ch->CFCR, DMA_CFCR_TOF);                             /* clear TO  */             /* :contentReference[oaicite:21]{index=21} */
	if (sr & DMA_CSR_USEF)
	{
		if (user_code_error < 2u)
		{
			WRITE_REG(ch->CFCR, DMA_CFCR_USEF);
			WRITE_REG(ch->CLBAR, (uint32_t)g_adc_mgr->chx_lli);        /* LBA = baza 4KB = adres węzła */
			WRITE_REG(ch->CLLR,  DMA_CLLR_ULL                 /* update link addr z węzła    */
						   |  DMA_CLLR_USA                /* ZAŁADUJ SAR                 */
						   |  DMA_CLLR_UDA                /* ZAŁADUJ DAR                 */
						   |  DMA_CLLR_UB1                /* ZAŁADUJ BR1                 */
						   |  DMA_CLLR_UT1                /* ZAŁADUJ TR1                 */
						   |  DMA_CLLR_UT2);              /* ZAŁADUJ TR2                 */
			SET_BIT(ch->CCR, DMA_CCR_EN);
			user_code_error++;
		}
		else
		{
		  Error_Handler();
		}
	//		  /* clear USE */             /* :contentReference[oaicite:22]{index=22}turn20file2 */
	}
  /* USER CODE END GPDMA1_Channel7_IRQn 0 */
//  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel7);
  /* USER CODE BEGIN GPDMA1_Channel7_IRQn 1 */

  /* USER CODE END GPDMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
//	ADC_MarkReady(m); //-> when TIM2 (CNV) is finished
//	if (ADC_FetchReady(m)){
//		m->ready_mask = 0;
	TIM4->CNT = 0;
	TIM8->CNT = 0;
	__HAL_TIM_ENABLE(&htim4); // CEN=1 → jeden okres (OPM)
	g_adc_mgr->common_ptr++;
//	}
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USB FS global interrupt.
  */
void USB_DRD_FS_IRQHandler(void)
{
  /* USER CODE BEGIN USB_DRD_FS_IRQn 0 */

  /* USER CODE END USB_DRD_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_DRD_FS);
  /* USER CODE BEGIN USB_DRD_FS_IRQn 1 */

  /* USER CODE END USB_DRD_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
