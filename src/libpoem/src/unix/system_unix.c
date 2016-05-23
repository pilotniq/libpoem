//
//  system_unix.c
//  bleGPS
//
//  Created by Erland Lewin on 2016-05-19.
//
//

#include <assert.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

#include <poem/system.h>

#define MAX_FDs 32
static struct pollfd fds[ MAX_FDs ];
static int fdCount = 0;

void system_waitForEvent( void )
{
  int err;
  
  if( fdCount == 0 )
    usleep(100); // sleep 100 microseconds
  else
  {
    TimeoutTime timeout;
    int timeoutMs;
    
    // establish next timeout
    timeout = system_timeout_getFirst();
    
    if( timeout == NULL )
    {
      timeoutMs = -1;
      // printf( "system_waitForEvent: no timeout\n" );
    }
    else
    {
      uint32_t now;
      
      now = system_timeMs_get();

      // printf( "system_waitForEvent: timeout time=%d, now=%d\n", timeout->time, now );

      if( system_timeMs_isBefore( timeout->time, now ) )
        return; // timeout has occurred
      else
        timeoutMs = timeout->time - now;
    }
    
    err = poll( fds, fdCount, timeoutMs);
  }
}

void system_addFD( int fd )
{
  fds[ fdCount ].fd = fd;
  // fds[ fdCount ].events = POLLHUP | POLLIN | POLLOUT | POLLPRI | POLLRDBAND | POLLRDNORM | POLLWRBAND;
  fds[ fdCount ].events = POLLHUP | POLLIN | POLLPRI | POLLRDBAND | POLLRDNORM;
  
  fdCount++;
}

uint32_t system_timeMs_get(void)
{
  struct timeval timeval;
  int err;
  uint32_t result;
  uint32_t msPart;
  uint64_t sPart;
  err = gettimeofday( &timeval, NULL );
  assert( err == 0 );
  
  // C standard says uint32_t values don't overflow, they are reduced module size of type.
  msPart = timeval.tv_usec / 1000;
  sPart = (timeval.tv_sec & 0xffffff) * ((uint32_t)1000);
  // sPart = sPart & 0xffffffff;
  result = sPart + msPart;
  // result = ((uint32_t) timeval.tv_usec / 1000) + (uint32_t) timeval.tv_sec * 1000;
  
  return result;
}
