/*
 * Erl's generic i2c library, implementation for Nordic Semiconductor's nRF51822
 */

#include "nrf_drv_twi.h"

#include "i2c.h"
#include "i2c_nrf51822.h"

static sError error;
int i2c_err_module_nr;

ret_code_t nrf51822_i2c_init( I2Cchannel i2cChannel,
			      nrf_twi_frequency_t frequency, 
			      uint8_t interruptPriority, 
			      uint32_t sclPin, uint32_t sdaPin,
			      nrf_drv_twi_evt_handler_t event_handler,
			      void *pContext )
{
  nrf_drv_twi_config_t config = { .frequency = frequency, 
                                  .interrupt_priority = interruptPriority,
                                  .scl = sclPin,
				  .sda = sdaPin };
  ret_code_t retCode;
  // nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE( MASTER_TWI_INST );

  // i2cChannel->twi = twi;

  retCode = nrf_drv_twi_init( &(i2cChannel->twi), &config, event_handler,
			      pContext );

  if( retCode != NRF_SUCCESS )
    goto FAIL;

  nrf_drv_twi_enable( &(i2cChannel->twi) );

  i2cChannel->errorModule = err_registerModule();
  error.module = i2cChannel->errorModule;
  error.moduleName = "nRF5 i2c dev";

 FAIL:
  return retCode;
}

int i2c_getErrorModule( I2Cchannel channel )
{
  return channel->errorModule;
}

static Error setError( ret_code_t retCode )
{
  switch( retCode )
  {
    case NRF_SUCCESS:
      return NULL;

    case NRF_ERROR_NRF_ERROR_BUSY:
      error.error = I2C_ERR_OTHER;
      error.description = "Busy (driver is not ready for a new transfer)";
      break;

    case NRF_ERROR_NRF_ERROR_INTERNAL:
      error.error = I2C_ERR_OTHER;
      error.description = "Internal (error detected by hardware)";
      break;

    case NRF_ERROR_INVALID_ADDR:
      error.error = I2C_ERR_OTHER;
      error.description = "Invalid addr (EasyDMA is used and memory adress in not in RAM)";
      break;

    case NRF_ERROR_DRV_TWI_ERR_OVERRUN:
      error.error = I2C_ERR_OTHER;
      error.description = "Overrun (unread data was replaced by new data)";
      break;

    case NRF_ERROR_DRV_TWI_ERR_ANACK:
      error.error = I2C_ERR_NACK_AFTER_ADDRESS;
      error.description = "NACK after address";
      break;

    case NRF_ERROR_DRV_TWI_ERR_DNACK:
      error.error = I2C_ERR_NACK_AFTER_DATA;
      error.description = "NACK after data byte";
      break;

    default:
      error.error = I2C_ERR_OTHER;
      error.description = "Unknown";
      break;  
  }
  error.cause = NULL;

  return &error;
}

Error i2c_read( I2Cchannel i2cChannel, uint8_t address, 
                uint32_t length, uint8_t *data )
{
  ret_code_t retCode;

  retCode = nrf_drv_twi_rx( &(i2cChannel->twi), address, data, length, 0 );
  if( retCode == NRF_SUCCESS )
    return NULL;
  else
    return setError( retCode );
}

Error i2c_write( I2Cchannel i2cChannel, uint8_t address, uint32_t length, uint8_t *data )
{
  ret_code_t retCode;

  retCode = nrf_drv_twi_tx( &(i2cChannel->twi), address, data, length, 0 );

  if( retCode == NRF_SUCCESS )
    return NULL;
  else
    return setError( retCode );
}
