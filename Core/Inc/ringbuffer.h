/*
 * ringbuffer.h
 *
 *  Created on: Aug 23, 2025
 *      Author: Celelele
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#include "main.h"
#include "string.h"

#define RB_MAX_SIZE 512

typedef enum
{
	RB_OK       	= 0x00U,
	RB_EMPTY    	= 0x01U,
	RB_FULL     	= 0x02U,
	RB_ERROR		= 0x03
} RingBuffer_StatusTypeDef;

typedef struct RingBuffer {
	uint16_t head;
	uint16_t tail;
	uint8_t *buf;
	uint16_t size;
} RingBuffer;

RingBuffer_StatusTypeDef RingBuffer_Init(RingBuffer* ring_buffer, uint8_t* buffer, size_t buffer_size);
RingBuffer_StatusTypeDef RingBuffer_Read(RingBuffer* ring_buffer, uint8_t* rx_buf);
size_t RingBuffer_PeekBlock(RingBuffer *ring_buffer, uint8_t *dst, size_t maxlen);
size_t RingBuffer_Consume(RingBuffer *ring_buffer, size_t n);
RingBuffer_StatusTypeDef RingBuffer_Write(RingBuffer* ring_buffer, uint8_t data);
size_t RingBuffer_WriteString(RingBuffer* ring_buffer, const char *text, size_t size);
RingBuffer_StatusTypeDef RingBuffer_Flush(RingBuffer* ring_buffer);
RingBuffer_StatusTypeDef RingBuffer_IsEmpty(RingBuffer* ring_buffer);
RingBuffer_StatusTypeDef RingBuffer_IsFull(RingBuffer* ring_buffer);

#endif /* INC_RINGBUFFER_H_ */
