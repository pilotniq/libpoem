/*
 *  
 */

#include "nrf_twi_mngr.h"
#include <poem/i2c_master.h>
#include <poem/nRF5/i2c_twiManager.h>

/*
 * static function prototypes
 */
static Error setError( ret_code_t retCode );

/*
 * static variables
 */
static sError error;
// a bit of a hack
// nrf_twi_mngr_t twi_template = NRF_DRV_TWI_INSTANCE(0);
// can't use SPI with instance 0 and TWI with instance 0, so use 1 for TWI.
NRF_TWI_MNGR_DEF(twi_template, 5 /* MAX_PENDING_TRANSACTIONS */, 1 /* TWI_INSTANCE_ID */);


/*
 * start of code
 */
void nrf5_i2c_twi_init( I2Cchannel i2cChannel,
			                nrf_twi_frequency_t frequency, 
			      uint8_t interruptPriority, 
			      uint32_t sclPin, uint32_t sdaPin,
                  bool clear_bus_init, bool hold_bus_uninit,
			      nrf_drv_twi_evt_handler_t event_handler
			      )
{
    // nrf_drv_twi_config_t config;
    ret_code_t err_code;
    // NRF_TWI_MNGR_DEF(twi, 5 /* MAX_PENDING_TRANSACTIONS */, 0 /* TWI_INSTANCE_ID */);

    i2cChannel->config.scl = sclPin;
    i2cChannel->config.sda = sdaPin;
    i2cChannel->config.frequency = frequency;
    i2cChannel->config.interrupt_priority = interruptPriority;
    i2cChannel->config.clear_bus_init = clear_bus_init;
    i2cChannel->config.hold_bus_uninit = hold_bus_uninit;

    memcpy( &(i2cChannel->twi), &twi_template, sizeof( twi_template ) );

    err_code = nrf_twi_mngr_init( &(i2cChannel->twi), &(i2cChannel->config) );

    if( err_code == NRF_SUCCESS )
    {
        i2cChannel->errorModule = err_registerModule();
        error.module = i2cChannel->errorModule;
        error.moduleName = "I2C Master nRF5 TWI Manager";
    }
}

// TODO: Make error handling generic
int i2c_getErrorModule( I2Cchannel channel )
{
    return channel->errorModule;
}

Error i2c_read( I2Cchannel channel, uint8_t address, uint32_t length, uint8_t *data )
{
    ret_code_t err_code;
    nrf_twi_mngr_transfer_t transfer;

    transfer.p_data = data;
    transfer.length = length;
    transfer.operation = address << 1 | 1;
    transfer.flags = 0; // NRF_TWI_MNGR_NO_STOP

    err_code = nrf_twi_mngr_perform( &(channel->twi), &(channel->config), 
                                     &transfer, 1, NULL );

    return setError( err_code );
}

Error i2c_write( I2Cchannel channel, uint8_t address, uint32_t length, 
                const uint8_t *data, bool stopAfter )
{
    ret_code_t err_code;
    nrf_twi_mngr_transfer_t transfer;

    transfer.p_data = data;
    transfer.length = length;
    transfer.operation = address << 1;
    transfer.flags = stopAfter ? NRF_TWI_MNGR_NO_STOP : 0;

    err_code = nrf_twi_mngr_perform( &(channel->twi), &(channel->config), 
                                     &transfer, 1, NULL );

    return setError( err_code );

}

static Error setError( ret_code_t retCode )
{
  switch( retCode )
  {
    case NRF_SUCCESS:
      return NULL;

    case NRF_ERROR_BUSY:
      error.error = I2C_ERR_OTHER;
      error.description = "Busy (driver is not ready for a new transfer)";
      break;

    case NRF_ERROR_INTERNAL:
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