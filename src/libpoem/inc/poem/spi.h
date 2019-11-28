/*
 * Erl's generic SPI interface
 */

#ifndef POEM_SPI_H
#define POEM_SPI_H

#include <stdbool.h>
#include <poem/error.h>

typedef struct sSPIchannel *SPIchannel;

typedef enum { SPIMODE_0, SPIMODE_1, SPIMODE_2, SPIMODE_3 } SPImode;

// Error spi_beginTransaction( SPIChannel channel, int maxSpeed, bool_t msbFirst, SPImode mode );
// Error spi_endTransaction( SPIChannel channel );

Error spi_transfer( SPIchannel channel, 
		    const uint8_t *outBuffer, size_t outBufferLength, 
		    uint8_t *inBuffer, size_t inBufferLength );

#endif
