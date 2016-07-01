/*
 * serial_osx.h
 */

#ifndef EMBLIB_SERIAL_APP_NRF5_H
#define EMBLIB_SERIAL_APP_NRF5_H

#include <poem/error.h>
#include <poem/serial.h>

// serialCHannel in practice not used, we only have one channel in HW.
typedef struct sSerialChannel
{
} sSerialChannel;

Error nrf51822_app_serial_init( SerialChannel channel,
				uint32_t baudRate,
				uint8_t interruptPriority, 
				int txdPin, int rxdPin,
				int ctsPin, int rtsPin );

#endif
