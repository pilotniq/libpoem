/**
 * @file circularBuffer.h
 * 
 * @author Erland Lewin <erland@lewin.nu>
 * 
 * Adapted from the voxiUtil library module of the same name.
 *
 * Semantics:
 * 
 * A circular buffer is a buffer of a limited length. If elements are written 
 * (pushed) to it when it is full, the push function returns false.
 * 
 * It was initially written for push-to-talk / speech detection code.
 * 
 * @remarks The circularBuffer is not thread-safe; the application code must ensure
 * that locks are used if the buffer is to be used from different threads.
 */

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

typedef struct sCircularBuffer
{
  int size;
  int headIndex;
  int tailIndex;
  int elementCount;
  uint8_t *buffer;
} sCircularBuffer, *CircularBuffer;

void circBuf_create( int size, CircularBuffer circularBuffer, uint8_t *buffer );

/*
 * returns false if the circular buffer is full, in which case the element will not 
 * have been written.
 */
bool circBuf_pushByte( CircularBuffer circBuf, uint8_t element );

/**
 *  Removes all the elements from the circular buffer. The circular buffer
 *  will be empty after this call.
 */
void circBuf_clear( CircularBuffer circBuf );

#if 0 /* Not Implemented Yet */
Error circBuf_popHead( CircularBuffer circBuf, void **element );
#endif

/* Remove and return the last (tail, oldest) element in the buffer. 
   It is not destroyed. */
uint8_t circBuf_popTailByte( CircularBuffer circBuf );

int circBuf_getElementCount( CircularBuffer circBuf );

#endif

