/*
 * Erl's generic i2c library, implementation for Nordic Semiconductor's nRF51822
 */

#ifndef I2C_TWIMANAGER_H
#define I2C_TWIMANAGER_H

#include "nrf_twi_mngr.h"

#include <poem/i2c_master.h>

typedef struct sI2Cchannel
{ 
  int errorModule;
  nrf_drv_twi_config_t config;
  nrf_twi_mngr_t twi;
} sI2Cchannel;

void nrf5_i2c_twi_init( I2Cchannel i2cChannel,
			      nrf_twi_frequency_t frequency, 
			      uint8_t interruptPriority, 
			      uint32_t sclPin, uint32_t sdaPin,
                  bool clear_bus_init, bool hold_bus_uninit,
			      nrf_drv_twi_evt_handler_t event_handler
			      );

#endif
