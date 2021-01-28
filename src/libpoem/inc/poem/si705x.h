/*
   Silicon Labs Si705x temperature sensor chips
   */
#ifndef POEM_SI705X_H
#define POEM_SI705X_H

#include <stdint.h>

#include <poem/error.h>
#include <poem/i2c_master.h>

typedef enum { SI705X_11_BIT = 3, 
               SI705X_12_BIT = 2, 
               SI705X_13_BIT = 1, 
               SI705X_14_BIT = 0 } Si705x_Resolution;

// below contains 100 x the temperature in Celcius
typedef int16_t Si705x_Temperature;

void si705x_init();
Error si705x_readSerial( I2Cchannel channel, uint8_t *serial );
Error si705x_setResolution( I2Cchannel channel, Si705x_Resolution resolution );
Error si705x_readNoHold( I2Cchannel channel, Si705x_Temperature *temperature );

#endif
