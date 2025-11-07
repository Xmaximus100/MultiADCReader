/*
 * ring_buffer->c
 *
 *  Created on: Aug 23, 2025
 *      Author: Celelele
 */

#include "ringbuffer.h"



static inline uint16_t RB_NextIdx(const RingBuffer *ring_buffer, uint16_t i){
    ++i; if (i == ring_buffer->size) i = 0;
    return i;
}

static inline uint16_t RB_Count(const RingBuffer *ring_buffer){
    return (uint16_t)((ring_buffer->head + ring_buffer->size - ring_buffer->tail) % ring_buffer->size);
}

static inline uint16_t RB_Room(const RingBuffer *ring_buffer){
    return (uint16_t)((ring_buffer->size - 1u) - RB_Count(ring_buffer));
}


RingBuffer_StatusTypeDef RingBuffer_Init(RingBuffer* ring_buffer, uint8_t *buffer){
    ring_buffer->head = 0;
    ring_buffer->tail = 0;
    ring_buffer->size = RB_MAX_SIZE;     // pełny rozmiar tablicy buf[]
    ring_buffer->buf = buffer;
    return RB_OK;
}

RingBuffer_StatusTypeDef RingBuffer_Flush(RingBuffer* ring_buffer){
    ring_buffer->head = 0;
    ring_buffer->tail = 0;
    return RB_OK;
}

RingBuffer_StatusTypeDef RingBuffer_IsEmpty(RingBuffer* ring_buffer){
    return (ring_buffer->tail == ring_buffer->head) ? RB_EMPTY : RB_OK;
}

/*
 * Effective check of next head with precalculated value of head
 */
static inline bool RingBuffer_IsFullFast(const RingBuffer* ring_buffer, uint16_t *next_head_out){
    uint16_t nh = RB_NextIdx(ring_buffer, ring_buffer->head);
    if (next_head_out) *next_head_out = nh;
    return (nh == ring_buffer->tail);
}

RingBuffer_StatusTypeDef RingBuffer_IsFull(RingBuffer* ring_buffer){
    return RingBuffer_IsFullFast(ring_buffer, NULL) ? RB_FULL : RB_OK;
}

RingBuffer_StatusTypeDef RingBuffer_Read(RingBuffer* ring_buffer, uint8_t* rx_buf){
    if (RingBuffer_IsEmpty(ring_buffer) == RB_EMPTY) return RB_EMPTY;
    *rx_buf = ring_buffer->buf[ring_buffer->tail];
    ring_buffer->tail = RB_NextIdx(ring_buffer, ring_buffer->tail);
    return RB_OK;
}

// --- PEEK: kopiuje do dst do maxlen bajtów, NIE przesuwa tail ---
size_t RingBuffer_PeekBlock(RingBuffer *ring_buffer, uint8_t *dst, size_t maxlen){
    uint16_t avail = RB_Count(ring_buffer);
    if (avail == 0 || maxlen == 0) return 0;

    size_t to_copy = (avail < maxlen) ? avail : maxlen;

    // ile ciągłych bajtów do końca tablicy od tail
    uint16_t first = (uint16_t)(ring_buffer->size - ring_buffer->tail);
    if (first > to_copy) first = (uint16_t)to_copy;

    // 1) pierwszy fragment
    memcpy(dst, &ring_buffer->buf[ring_buffer->tail], first);

    // 2) ewentualny drugi fragment (po zawinięciu)
    size_t remaining = to_copy - first;
    if (remaining){
        memcpy(dst + first, &ring_buffer->buf[0], remaining);
    }

    return to_copy;
}

// --- CONSUME: przesuwa tail o n (zużywa dane), zwraca faktycznie zużyte bajty ---
size_t RingBuffer_Consume(RingBuffer *ring_buffer, size_t n){
    uint16_t avail = RB_Count(ring_buffer);
    if (avail == 0) return 0;

    size_t consume = (n < avail) ? n : avail;

    // przesuwanie bez kosztownego modulo
    uint16_t cap   = ring_buffer->size;
    uint16_t tail  = ring_buffer->tail;

    uint16_t first = (uint16_t)(cap - tail);
    if (first > consume) first = (uint16_t)consume;

    tail = (uint16_t)(tail + first);
    if (tail == cap) tail = 0;

    size_t remaining = consume - first;
    if (remaining){
        // teraz tail == 0, więc wystarczy dodać remaining
        tail = (uint16_t)remaining;               // remaining < cap zawsze
    }

    ring_buffer->tail = tail;
    return consume;
}

RingBuffer_StatusTypeDef RingBuffer_Write(RingBuffer* ring_buffer, uint8_t data){
    uint16_t next_head;
    if (RingBuffer_IsFullFast(ring_buffer, &next_head)) return RB_FULL;  // bez podwójnych obliczeń
    ring_buffer->buf[ring_buffer->head] = data;
    ring_buffer->head = next_head;
    return RB_OK;
}

size_t RingBuffer_WriteString(RingBuffer* ring_buffer, const char *text, size_t size){
    size_t ptr = 0;
    while (ptr < size){ //text[ptr] &&
        if (RingBuffer_Write(ring_buffer, (uint8_t)text[ptr]) != RB_OK) break;
        ++ptr;
    }
    return ptr;
}


