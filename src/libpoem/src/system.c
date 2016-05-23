//
//  system.c
//  bleGPS
//
//  Created by Erland Lewin on 2016-05-20.
//
//
#include <assert.h>
#include <stdio.h> // just for debugging
#include <unistd.h>

#include <poem/system.h>

// TODO: Abstract to generic linked list
TimeoutTime firstTimeout, lastTimeout;

void system_init( void )
{
  firstTimeout = NULL;
  lastTimeout = NULL;
}

void system_timeout_register( TimeoutTime timeout )
{
  TimeoutTime timeoutAfter;
  
  printf( "Timeout: registering %p\n", timeout );
  
  for( timeoutAfter = firstTimeout;
      (timeoutAfter != NULL) && system_timeMs_isBefore( timeoutAfter->time, timeout->time );
      timeoutAfter = timeoutAfter->next )
    ;
  
  // here, timeAfter will be the timeout nearest after the parameter
  if( timeoutAfter == NULL )
  {
    if( lastTimeout == NULL )
    {
      assert( firstTimeout == NULL );
      firstTimeout = timeout;
      lastTimeout = timeout;
      timeout->prev = NULL;
      timeout->next = NULL;
    }
    else
    {
      lastTimeout->next = timeout;
      timeout->prev = lastTimeout;
      timeout->next = NULL;
      lastTimeout = timeout;
    }
  }
  else
  {
    if( timeoutAfter->prev == NULL )
    {
      assert( firstTimeout == timeoutAfter);
      firstTimeout = timeout;
      timeout->prev = NULL;
      timeout->next = timeoutAfter;
    }
    else
    {
      timeoutAfter->prev->next = timeout;
      timeout->prev = timeoutAfter->prev;
      timeout->next = timeoutAfter;
      timeoutAfter->prev = timeout;
    }
  }
}

/*
    Return true if t1 is before t2
 */
bool system_timeMs_isBefore( uint32_t t1, uint32_t t2 )
{
  return (t2 - t1) < (UINT32_MAX / 2);
}

void system_timeout_unregister( TimeoutTime timeout )
{
  printf( "timoeut: unregistering timeout %p\n", timeout );
  
  if( timeout->prev == NULL )
  {
    assert( firstTimeout == timeout );
  
    firstTimeout = timeout->next;
  }
  else
  {
    assert( timeout->prev->next == timeout );
    
    timeout->prev->next = timeout->next;
  }
  
  if( timeout->next == NULL )
  {
    assert( lastTimeout == timeout );
    
    lastTimeout = timeout->prev;
  }
  else
  {
    assert( timeout->next->prev == timeout );
    
    timeout->next->prev = timeout->prev;
  }
}

bool system_timeout_expired( TimeoutTime timeout )
{
  uint32_t now = system_timeMs_get();
  
  if( system_timeMs_isBefore( now, timeout->time ) )
  {
    // now is before time, has not expired
    return false;
  }
  else
  {
    // remove timeout
    system_timeout_unregister( timeout );
    
    return true;
  }
}

TimeoutTime system_timeout_getFirst( void )
{
  return firstTimeout;
}

