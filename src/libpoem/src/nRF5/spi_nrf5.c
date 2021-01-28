#include <poem/error.h>
#include <poem/nrf5/spi_nrf5.h>
#include <poem/nrf5/error_nRF5.h>

#include "sdk_config.h"
#include "nrf_drv_spi.h"

static int errorModule = -1;
static sError error;

static void check_initialize(void);

static void check_initialize(void)
{
  if( errorModule == -1 )
    errorModule = err_registerModule();
}

Error nrf5_spi_init( SPIchannel channel, 
		     int instance, uint8_t sclkPin, uint8_t mosiPin, uint8_t misoPin, 
		     nrf_drv_spi_frequency_t frequency, nrf_drv_spi_mode_t mode, nrf_drv_spi_bit_order_t bitOrder )
{
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  nrf_drv_spi_t inst0 = NRF_DRV_SPI_INSTANCE(0);
  ret_code_t retCode;

  check_initialize();
  
  switch( instance )
  {
    case 0:
      channel->instance = inst0;
      break;
/*
    case 1:
      channel->instance = NRF_DRV_SPI_INSTANCE(1);
      break;
*/
    default:
      error.module = errorModule;
      error.error = 1;
      error.description = "Invalid SPI Instance";
      error.moduleName = "NRF5 SPI";

      return &error;
  }

  spi_config.sck_pin = sclkPin;
  spi_config.mosi_pin = mosiPin;
  spi_config.miso_pin = misoPin;
  spi_config.ss_pin = NRFX_SPI_PIN_NOT_USED;

  spi_config.frequency = frequency;
  spi_config.mode = mode;
  spi_config.bit_order = bitOrder;
  
  retCode = nrf_drv_spi_init( &(channel->instance), &spi_config, NULL, channel );
  
  if( retCode == NRF_SUCCESS )
    return NULL;
  else
  {
    nRF5_makeError( &error, retCode );
    return &error;
  }
}

Error spi_transfer( SPIchannel channel, 
		    const uint8_t *outBuffer, size_t outBufferLength, 
		    uint8_t *inBuffer, size_t inBufferLength )
{
  ret_code_t retCode;

  // todo: check lengtsh of buffers are below 255, max supported by nRF
  retCode = nrf_drv_spi_transfer( &(channel->instance), outBuffer, outBufferLength, inBuffer, inBufferLength );
  
  if( retCode == NRF_SUCCESS )
    return NULL;
  else
  {
    nRF5_makeError( &error, retCode );
    return &error;
  }
}
