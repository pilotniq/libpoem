#include <string.h>  // for memcpy
#include <poem/i2c.h>

#include <poem/nRF5/error_nRF5.h>
#include <poem/nRF5/i2c_app_nRF5.h>

#include "app_util_platform.h" // from nRF SDK app_common

#define QUEUE_SIZE 15

// typedef sI2Cchannel app_twi_t;

// static app_twi_t app_twi;
static int errorModule = -1;
static app_twi_transaction_t const *queue_buffer[ QUEUE_SIZE + 1 ];
static sError error;

/*
 * Start of code
 */
Error nrf51822_app_i2c_init( I2Cchannel i2cChannel,
			     uint32_t /* nrf_twi_frequency_t */frequency, 
			     /* uint8_t interruptPriority, */
			     uint32_t sclPin, uint32_t sdaPin )
{
  nrf_drv_twi_config_t twi_config;
  app_twi_t template = APP_TWI_INSTANCE( 0 );
  nrf_twi_frequency_t freq;
  ret_code_t retCode;

  if( errorModule == -1 )
    errorModule = err_registerModule();

  switch( frequency )
  {
    case 100000:
      freq = TWI_FREQUENCY_FREQUENCY_K100;
      break;

    case 250000:
      freq = TWI_FREQUENCY_FREQUENCY_K250;
      break;

    case 400000:
      freq = TWI_FREQUENCY_FREQUENCY_K400;
      break;

    default:
      error.module = errorModule;
      error.error = I2C_ERR_INVALID_FREQUENCY;
      error.cause = NULL;
      error.moduleName = "nRF i2c app";
      error.description = "Invalid i2c frequency";
      
      return &error;
  }

  // clean up later
  memcpy( &(i2cChannel->channel), &template, sizeof( template ) );
  // i2cChannel->channel = template;

  twi_config.frequency = freq;
  twi_config.interrupt_priority = APP_IRQ_PRIORITY_LOW; // hard code for now, make parameter if neccessary
  twi_config.scl = sclPin;
  twi_config.sda = sdaPin;

  // i2cChannel->channel.twi = NRF_DRV_TWI_INSTANCE( 0 );

  retCode = app_twi_init( &(i2cChannel->channel), &twi_config, QUEUE_SIZE, queue_buffer );
  if( retCode != NRF_SUCCESS )
  {
    nRF5_makeError( &error, retCode );
    return &error;
  }

  return NULL;
}

Error i2c_write( I2Cchannel i2cChannel, uint8_t address, uint32_t length, const uint8_t *data, bool stopAfter )
{
  app_twi_transfer_t transfer;
  ret_code_t retCode;

  transfer.p_data = (uint8_t *) data;
  transfer.length = length;
  transfer.operation = APP_TWI_WRITE_OP( address );
  transfer.flags = stopAfter ? 0 : APP_TWI_NO_STOP;

  retCode = app_twi_perform( &(i2cChannel->channel), &transfer, 1, NULL );

  if( retCode == NRF_SUCCESS )
    return NULL;
  else
  {
    nRF5_makeError( &error, retCode );
    
    return &error;
  }
}

Error i2c_read( I2Cchannel i2cChannel, uint8_t address, uint32_t length, uint8_t *data )
{
  app_twi_transfer_t transfer;
  ret_code_t retCode;

  transfer.p_data = data;
  transfer.length = length;
  transfer.operation = APP_TWI_READ_OP( address );
  transfer.flags = 0; // stopAfter ? 0 : APP_TWI_NO_STOP;

  retCode = app_twi_perform( &(i2cChannel->channel), &transfer, 1, NULL );

  if( retCode == NRF_SUCCESS )
    return NULL;
  else
  {
    nRF5_makeError( &error, retCode );
    
    return &error;
  }
}
