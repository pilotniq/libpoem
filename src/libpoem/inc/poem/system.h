//
//  system.h
//  bleGPS
//
//  Created by Erland Lewin on 2016-05-19.
//
//

#ifndef POEM_SYSTEM_H
#define POEM_SYSTEM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct sTimeoutTime
{
  struct sTimeoutTime *prev, *next;
  uint32_t time;
} sTimeoutTime, *TimeoutTime;

void system_init( void );

// Arbitrary time in milliseconds
uint32_t system_timeMs_get(void);
bool system_timeMs_isBefore( uint32_t t1, uint32_t t2 );

void system_timeout_register( TimeoutTime timeout );
void system_timeout_unregister( TimeoutTime timeout );
bool system_timeout_expired( TimeoutTime timeout );
TimeoutTime system_timeout_getFirst( void );

void system_waitForEvent( void );

#endif /* system_h */