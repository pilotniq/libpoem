//
//  gps_ublox6.h
//  bleGPS
//
//  Created by Erland Lewin on 2016-05-18.
//
//

#ifndef POEM_GPS_UBLOX6_H
#define POEM_GPS_UBLOX6_H

#include <poem/error.h>
#include <poem/serial.h>

typedef void (*GPSReadyCallback)( Error error );

void gps_ublox6_init( SerialChannel serial, GPSReadyCallback readyCallback );

#endif /* gps_ublox6_h */
