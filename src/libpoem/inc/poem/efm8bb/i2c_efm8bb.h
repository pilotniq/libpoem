/*
 * Erl's generic i2c library, implementation for Silicon Labs EFM88BB1 controllers
 */

#ifndef I2C_EFM8BB1_H
#define I2C_EFM8BB1_H

#include <stdint.h>
#include <poem/error.h>
#include <poem/i2c.h>

/*
 * state sequence. After received, putting data
 */
typedef enum { EFM8BB_I2C_SLAVE_STATE_IDLE, EFM8BB_I2C_SLAVE_STATE_RECEIVING,
               EFM8BB_I2C_SLAVE_STATE_RECEIVED,
			   EFM8BB_I2C_SLAVE_STATE_GOT_READ,
			   EFM8BB_I2C_SLAVE_STATE_SENDING } EFM8BB1_i2c_slave_state;

typedef struct sI2Cchannel
{ 
  uint8_t slaveAddress;
  uint8_t *inputBuffer;

  // below should be bool instead of int, but doesn't work on C51 compiler
  int outputBufferReady;
  int outputBufferSize;
  int outputBufferCursor;
  uint8_t *outputBuffer;

  int inputBufferSize;
  int inputBufferCursor;
  EFM8BB1_i2c_slave_state state;
} sI2Cchannel;

Error efm8bb_i2c_slave_init( I2Cchannel i2cChannel, uint8_t address,
							 uint8_t *inputBuffer, int inputBufferSize );
bool i2c_slave_service( I2Cchannel i2cChannel );
Error i2c_slave_write( I2Cchannel i2c, int byteCount, const uint8_t *buffer );
Error i2c_slave_getInputBufferCount( const sI2Cchannel *i2c, int *count );

#endif
