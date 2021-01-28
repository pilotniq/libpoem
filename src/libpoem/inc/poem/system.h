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

// Arbitrary time in milliseconds
uint32_t system_timeMs_get(void);
bool system_timeMs_isBefore( uint32_t t1, uint32_t t2 );

void system_timeout_register( TimeoutTime timeout );
void system_timeout_unregister( TimeoutTime timeout );
bool system_timeout_expired( TimeoutTime timeout );
TimeoutTime system_timeout_getFirst( void );

/*
 * If system_setEventPending has been called since the call to system_waitForEvent, 
 * then system_waitForEvent will return immediately 
 */
void system_setEventPending( void );
void system_waitForEvent( void );

// system_reset must be implemented by the application. It is called on reset and startup
// TODO: pass reason argument to detect if it was due to powerup, brownout,
void system_reset( void );

void system_delayMs( int ms );
void system_delayUs( int us );

#endif /* system_h */
