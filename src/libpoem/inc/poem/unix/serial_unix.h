/*
 * serial_osx.h
 */

#ifndef EMBLIB_SERIAL_UNIX_H
#define EMBLIB_SERIAL_UNIX_H

#include <termios.h>
#include <poem/error.h>

typedef struct sSerialChannel
{
  int fd;
} sSerialChannel;

Error unix_serial_init( SerialChannel channel, const char *device, speed_t baudRate );

#endif
