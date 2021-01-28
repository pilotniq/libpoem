/**  
  @file circularBuffer.c
  
  @author Erland Lewin <erl@voxi.com>
  
 Adapted from the voxiUtil library module by the same name. Semantics have been changed to not allow
 the buffer to overflow.
 
  Semantics:
  
  A circular buffer is a buffer of a limited length. If elements are written 
  (pushed) to it when it is full, the oldest element is removed from the buffer
  and destroyed (if a destroyFunc has been specified).
  
  It was initially written for the push-to-talk / speech detection code, to 
  buffer for a limited amount of time.
  
  It internally stores elements like this:
  
  Vector:
  
     0         1        2  3  4     5         6 
              tail                 head   
  element3  element4             element1  element2
*/
#define NDEBUG /* to debug C51 - if assert causes large memory use */
#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
/*
#include <voxi/debug.h>
#include <voxi/util/err.h>
#include <voxi/util/mem.h>
#include <voxi/util/vector.h>
*/
#include <poem/circularBuffer.h>

/*
 * Code
 */

void circBuf_create( int size, CircularBuffer circBuf, uint8_t *buffer )
{
  circBuf->size = size;
  circBuf->headIndex = 0;
  circBuf->tailIndex = 1;
  circBuf->elementCount = 0;
  circBuf->buffer = buffer;
}

// returns true on success, false if buffer is full.
bool circBuf_pushByte( CircularBuffer circBuf, uint8_t element )
{
  /* 
     if the buffer is full, destroy the last (tail) element.
     add the new element at the head.
  */
  if( circBuf->elementCount == circBuf->size )
    return false;
  
  if( circBuf->headIndex == (circBuf->size - 1) )
    circBuf->headIndex = 0;
  else
    circBuf->headIndex++;
  
  /* Write the new element */
  circBuf->buffer[ circBuf->headIndex ] = element;
  
  circBuf->elementCount++;
  
  assert( circBuf->elementCount <= circBuf->size );
  
  return true;
}

uint8_t circBuf_popTailByte( CircularBuffer circBuf )
{
  uint8_t element;
  
  assert( circBuf->elementCount > 0 );
  
  /* Remove and return the last (tail, oldest) element in the buffer. 
     It is not destroyed. */
  element = circBuf->buffer[ circBuf->tailIndex ];
  
  if( circBuf->tailIndex == (circBuf->size - 1))
    circBuf->tailIndex = 0;
  else
    circBuf->tailIndex++;
  
  circBuf->elementCount--;
  
  return element;
}

int circBuf_getElementCount( CircularBuffer circBuf )
{
  return circBuf->elementCount;
}

void circBuf_clear( CircularBuffer circBuf )
{
  while( circBuf->elementCount > 0 )
    circBuf_popTailByte( circBuf );
}

