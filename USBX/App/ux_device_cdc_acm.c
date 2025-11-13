/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
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
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ux_device_class_cdc_acm.h"
#include "app_usbx_device.h"
#include "at_parser.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	TX_IDLE = 0,
	TX_BUSY,
	TX_ERROR
} USB_Tx_typeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define APP_RX_DATA_SIZE   2048
#define APP_TX_DATA_SIZE   2048

/* Data length for vcp */
#define VCP_WORDLENGTH8  8
#define VCP_WORDLENGTH9  9

/* the minimum baudrate */
#define MIN_BAUDRATE     9600
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
UX_SLAVE_CLASS_CDC_ACM *cdc_acm = UX_NULL;

bool data_collected = false;
RingBuffer rb_tx;
RingBuffer rb_rx;
uint16_t successful_packets = 0;
uint32_t successful_transfers = 0;

static volatile uint8_t g_usb_tx_busy = 0;     // 0 = można wysyłać
static uint8_t g_usb_tx_frame[64];              // ramka USB FS (MaxPacket = 64 B)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */
	UX_PARAMETER_NOT_USED(cdc_acm_instance);
    cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;
  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
    UX_PARAMETER_NOT_USED(cdc_acm_instance);
    cdc_acm = UX_NULL;
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */

/**
  * @brief  USBD_CDC_ACM_Transmit
  *         This function allows to send data via USB
  * @param  buffer: input buffer with data, size: size of the buffer, sent: data sent successfully
  * @retval status: status of the transmission
  */
uint32_t USBD_CDC_ACM_Transmit(uint8_t* buffer, uint32_t size, uint32_t* sent){
    UINT retVal;
    if(cdc_acm!=NULL)
    {
    	uint8_t tx_state = TX_IDLE;
		do
		{
			switch(tx_state){
				case TX_IDLE:
					retVal = ux_device_class_cdc_acm_write_run(cdc_acm,buffer,size,sent);
					if (retVal == UX_STATE_WAIT) tx_state = TX_BUSY;
					break;
				case TX_BUSY:
					retVal = ux_device_class_cdc_acm_write_run(cdc_acm, UX_NULL, 0, sent);
				    if (retVal == UX_STATE_NEXT) {
				    	tx_state = TX_IDLE;
				    }
				    else if (retVal == UX_STATE_ERROR)
				    {
				    	tx_state = TX_ERROR;
				    	Error_Handler();
				    }
				    break;
			}
//			retVal = ux_device_class_cdc_acm_write_run(cdc_acm,buffer,size,sent);
		}
		while(UX_STATE_NEXT != retVal);
		successful_transfers++;
		return 0;
    }
    else
    {
      return 1;
    }
}

/**
  * @brief  USBD_CDC_ACM_Receive
  *         This function allows to receive data via USB
  * @param  buffer: input buffer for data, size: size of the buffer, received: data received successfully
  * @retval status: status of the transmission
  */
uint32_t USBD_CDC_ACM_Receive(uint8_t* buffer, uint32_t size, uint32_t* received){
    if(cdc_acm!=NULL)
    {
      ux_device_class_cdc_acm_read_run(cdc_acm,buffer,size,received);
      if (*received != 0)
      {
		RingBuffer_WriteString(&rb_rx, (const char*)buffer, *received);
		*received = 0;
		uint8_t msg;
		while (RingBuffer_Read(&rb_rx, &msg) == RB_OK){
			AT_ReadChar(msg);
		}
      }
      return 0;
    }
    else
    {
      return 1;
    }
}

// Pompa TX: spróbuj wysłać kolejny blok z ringu
uint32_t USB_TxPumpFromRing(RingBuffer *rb)
{
	if (cdc_acm == UX_NULL) return 1;
//    if (!USB_EpCanTx()) return;       // EP (Endpoint) zajęty
	uint32_t sent=0;

    size_t n = RingBuffer_PeekBlock(rb, g_usb_tx_frame, sizeof(g_usb_tx_frame));
    if (n == 0) return 0;

    if (USBD_CDC_ACM_Transmit(g_usb_tx_frame, (uint32_t)n, &sent) == 0){
        (void)RingBuffer_Consume(rb, n);   // zużyj dopiero po sukcesie
    }
    return 1;
}
/* USER CODE END 1 */
