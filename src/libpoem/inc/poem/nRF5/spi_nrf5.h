/*
 * nRF5 specific include file
 */
#ifndef POEM_NRF5_SPI_H
#define POEM_NRF5_SPI_H

#include "nrf_drv_spi.h"

typedef struct sSPIchannel
{
  nrf_drv_spi_t instance;
} sSPIchannel, *SPIchannel;

Error nrf5_spi_init( SPIchannel channel, int instance, 
                     uint8_t sclkPin, uint8_t mosiPin, uint8_t misoPin, 
		             nrf_drv_spi_frequency_t frequency, nrf_drv_spi_mode_t mode, nrf_drv_spi_bit_order_t bitOrder );

#endif