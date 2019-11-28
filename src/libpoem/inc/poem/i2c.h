/*
  Erl's i2c generic interface
*/

#ifndef EMBLIB_I2C_H
#define EMBLIB_I2C_H

#include <stdint.h>
#include <stdbool.h>

#include <poem/error.h>

// #include "sdk_errors.h" // for ret_code_t

// init function is chip specific
// void i2c_init(void);

typedef struct sI2Cchannel *I2Cchannel;
enum { I2C_ERR_INVALID_FREQUENCY };

// TODO: Make error handling generic
// Error i2c_read( I2Cchannel channel, uint8_t address, uint32_t length, uint8_t *data );
// Error i2c_write( I2Cchannel channel, uint8_t address, uint32_t length, const uint8_t *data,
// 		 bool stopAfter );

/*
 *  i2c slave api
 *
 *  init slave with address, and input buffer
 *  regularly poll i2c? NOP if irq-driven
 *  main loop checks if packet has arrived.
 *  if so, it returns data
 *
 */
//void i2c_slave_

#endif
