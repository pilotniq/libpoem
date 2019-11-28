/*
  Erl's embedded library 
*/

#ifndef EMBLIB_SERIAL
#define EMBLIB_SERIAL

#include <stdbool.h>
#include <stdint.h>
// #include <unistd.h> // for ssize_t
#include "error.h"

enum {
  SERIAL_ERR_INVALID_BAUDRATE
};

typedef struct sSerialChannel *SerialChannel;

/*
 * Start asynchronous write. Copies data from 'buffer' to internal buffer. 
 *
 * Parameter: length: input: amount of data in buffer. Output: actual bytes written, can be less if internal
 *   buffer is filled.
 *
 */
Error serial_write( SerialChannel channel, const uint8_t *buffer, int *length );

/*
 * Non-blocking read
 * 
 * Parameters: length: input: size of buffer. Output: number of bytes placed in buffer 
 * 
 */
Error serial_read( SerialChannel channel, uint8_t *buffer, int *length );

/*
 */
Error serial_writeFromBuffer( SerialChannel channel, const uint8_t *buffer, int bufferSize, int *cursor,
                              bool *done);

Error serial_readToBuffer( SerialChannel channel, uint8_t *buffer, int bufferSize, int *cursor,
                           bool *done);

#endif
