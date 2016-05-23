/*
  Portable Embedded Library (poem)

  gps.h
*/

#ifndef POEM_GPS_H
#define POEM_GPS_H

#include <stdint.h>
#include <stdbool.h>

#include <poem/error.h>

typedef enum
{
  GPS_ERR_NOT_READY,
} GPSErrorCode;

typedef struct
{
  int32_t longitude; // / 10^-7
  int32_t latitude;  // / 10^-7
  uint32_t accuracy; // mm
} sGPSPosition, *GPSPosition;

typedef void (*PositionCallback)( const sGPSPosition *position );

void gps_service( void );
// Call the callback with positions at approximately a 1 Hz callback rate
Error gps_getPositions( PositionCallback callback );
Error gps_power_set( bool on );

#endif
