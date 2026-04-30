#ifndef UTIL_FIFO_H
#define UTIL_FIFO_H

// Circular FIFO buffer implementation ported from BeRTOS.
// ATOMIC_BLOCK sections (from avr/atomic.h) are commented out: on ESP32,
// inter-task safety is handled by FreeRTOS at the call sites that need it.

#include <stddef.h>
#include "freertos/FreeRTOS.h"

// Shared spinlock for FIFO helper functions tagged as _locked.
// Defined in AFSK.cpp.
extern portMUX_TYPE g_fifo_mux;

typedef struct FIFOBuffer
{
  unsigned char *begin;
  unsigned char *end;
  unsigned char *head;
  unsigned char *tail;
} FIFOBuffer;

inline bool fifo_isempty(const FIFOBuffer *f) {
  return f->head == f->tail;
}

inline bool fifo_isfull(const FIFOBuffer *f) {
  return ((f->head == f->begin) && (f->tail == f->end)) || (f->tail == f->head - 1);
}

inline void fifo_push(FIFOBuffer *f, unsigned char c) {
  *(f->tail) = c;

  if (f->tail == f->end) {
    f->tail = f->begin;
  } else {
    f->tail++;
  }
}

inline unsigned char fifo_pop(FIFOBuffer *f) {
  if(f->head == f->end) {
    f->head = f->begin;
    return *(f->end);
  } else {
    return *(f->head++);
  }
}

inline void fifo_flush(FIFOBuffer *f) {
  f->head = f->tail;
}

inline bool fifo_isempty_locked(const FIFOBuffer *f) {
  bool result;
  taskENTER_CRITICAL(&g_fifo_mux);
  {
    result = fifo_isempty(f);
  }
  taskEXIT_CRITICAL(&g_fifo_mux);
  return result;
}

inline bool fifo_isfull_locked(const FIFOBuffer *f) {
  bool result;
  taskENTER_CRITICAL(&g_fifo_mux);
  {
    result = fifo_isfull(f);
  }
  taskEXIT_CRITICAL(&g_fifo_mux);
  return result;
}

inline void fifo_push_locked(FIFOBuffer *f, unsigned char c) {
  taskENTER_CRITICAL(&g_fifo_mux);
  {
    fifo_push(f, c);
  }
  taskEXIT_CRITICAL(&g_fifo_mux);
}

inline unsigned char fifo_pop_locked(FIFOBuffer *f) {
  unsigned char c;
  taskENTER_CRITICAL(&g_fifo_mux);
  {
    c = fifo_pop(f);
  }
  taskEXIT_CRITICAL(&g_fifo_mux);
  return c;
}

inline void fifo_init(FIFOBuffer *f, unsigned char *buffer, size_t size) {
  f->head = f->tail = f->begin = buffer;
  f->end = buffer + size -1;
}

inline size_t fifo_len(FIFOBuffer *f) {
  return f->end - f->begin;
}

#endif
