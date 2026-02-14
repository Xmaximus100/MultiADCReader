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
/*
 * USBD_CDC_ACM_Activate - Activate CDC ACM device instance
 * @param cdc_acm_instance: Pointer to CDC ACM class instance
 * @return none
 * 
 * Called by USBX when the CDC ACM device is activated (connected and configured).
 * Stores the CDC ACM instance pointer for use in transmit/receive operations.
 * This callback is invoked after successful USB enumeration and configuration.
 */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */
	UX_PARAMETER_NOT_USED(cdc_acm_instance);
    cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;
  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/*
 * USBD_CDC_ACM_Deactivate - Deactivate CDC ACM device instance
 * @param cdc_acm_instance: Pointer to CDC ACM class instance (unused)
 * @return none
 * 
 * Called by USBX when the CDC ACM device is deactivated (disconnected or suspended).
 * Clears the CDC ACM instance pointer to prevent operations on disconnected device.
 * This callback is invoked when USB cable is disconnected or device is suspended.
 */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
    UX_PARAMETER_NOT_USED(cdc_acm_instance);
    cdc_acm = UX_NULL;
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/*
 * USBD_CDC_ACM_ParameterChange - Handle CDC ACM parameter change requests
 * @param cdc_acm_instance: Pointer to CDC ACM class instance (unused)
 * @return none
 * 
 * Called by USBX when CDC ACM control parameters change (e.g., baud rate, line coding).
 * Currently unused but can be extended to handle parameter changes from the host.
 * This callback is invoked when the host sends SET_LINE_CODING or similar requests.
 */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */

/*
 * USBD_CDC_ACM_Transmit - Transmit data via USB CDC ACM interface
 * @param buffer: Pointer to data buffer to transmit
 * @param size: Size of data to transmit in bytes
 * @param sent: Output parameter for number of bytes actually sent
 * @return 0 on success, 1 if CDC ACM instance is not available
 * 
 * Transmits data through the USB CDC ACM interface. Handles state machine for
 * transmission (IDLE, BUSY, ERROR) and waits for completion. Tracks successful
 * transfers. Returns 0 on success, 1 if the device is not connected or configured.
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

/*
 * USBD_CDC_ACM_Receive - Receive data via USB CDC ACM interface
 * @param buffer: Pointer to buffer for received data
 * @param size: Maximum size of buffer in bytes
 * @param received: Output parameter for number of bytes actually received
 * @return 0 on success, 1 if CDC ACM instance is not available
 * 
 * Receives data from the USB CDC ACM interface. Reads available data into the buffer,
 * writes it to the RX ring buffer, and processes characters through the AT parser.
 * Returns 0 on success, 1 if the device is not connected or configured.
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

/*
 * USB_TxPumpFromRing - Pump data from ring buffer to USB transmit
 * @param rb: Pointer to ring buffer containing data to transmit
 * @return 1 if data was sent or attempted, 0 if buffer was empty, 1 if device unavailable
 * 
 * Attempts to transmit data from a ring buffer through the USB CDC ACM interface.
 * Reads up to 64 bytes (USB full-speed max packet size) from the ring buffer,
 * transmits it via USB, and removes the sent data from the buffer. This function
 * should be called periodically or from a task to maintain USB communication.
 */
uint32_t USB_TxPumpFromRing(RingBuffer *rb)
{
	if (cdc_acm == UX_NULL) return 1;
	uint32_t sent=0;

    size_t n = RingBuffer_PeekBlock(rb, g_usb_tx_frame, sizeof(g_usb_tx_frame));
    if (n == 0) return 0;

    if (USBD_CDC_ACM_Transmit(g_usb_tx_frame, (uint32_t)n, &sent) == 0){
        (void)RingBuffer_Consume(rb, n);   // consume data from ring buffer if transmission was successful
    }
    return 1;
}

uint32_t USB_TxZLP()
{
	uint32_t sent=0;
	USBD_CDC_ACM_Transmit(g_usb_tx_frame, 0, &sent);
	return 1;
}
/* USER CODE END 1 */
