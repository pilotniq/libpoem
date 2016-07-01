//
//  gpsTest.c
//  bleGPS
//
//  Created by Erland Lewin on 2016-05-18.
//
//

#include <assert.h>
#include <stdio.h>

#include <poem/gps.h>
#include <poem/error.h>
#include <poem/serial.h>
#include <poem/system.h>

#include <poem/gps_ublox6.h>
#include <poem/system.h>

#include <poem/unix/system_unix.h>

typedef enum { STATE_WAIT_FOR_READY,
       STATE_READY,
       STATE_RECEIVING_POSITIONS,
       STATE_OFF,
} AppState;

AppState state;
bool done, ready;
sTimeoutTime timeout;

/*
 * static function prptotypes
 */
static void gpsReadyCallback( Error error );
static void positionCallback( const sGPSPosition *position );
static void stateNext( AppState newState );
static void stateControl( void );

int main( int argc, char **argv )
{
  sSerialChannel serial;
  Error error;
  
  ready = false;
  system_init();
  
  error = unix_serial_init( &serial, argv[1], 9600 );
  if( error != NULL )
  {
    fprintf( stderr, "serial_init failed: %s (%d) %s\n", error->moduleName, error->error, error->description );
    return -1;
  }

  state = STATE_WAIT_FOR_READY;
  
  gps_ublox6_init( &serial, gpsReadyCallback );
  
  done = false;
  while( !done )
  {
    gps_service();
    stateControl();
    gps_service();
    system_waitForEvent();
  }
  
  return 0;
}

static void stateControl( void )
{
  switch( state )
  {
    case STATE_RECEIVING_POSITIONS:

      if( system_timeout_expired( &timeout ))
        stateNext( STATE_OFF );
      break;
      
    case STATE_OFF:
      if( system_timeout_expired( &timeout ))
      {
        printf( "Turning on GPS\n");
        gps_power_set( true );
      
        stateNext( STATE_WAIT_FOR_READY );
        // wait for ready callback
      }
      break;
      
    default:
      break;
      
  }
}

static void stateNext( AppState newState )
{
  Error error;
  
  state = newState;
  
  switch( newState )
  {
    case STATE_OFF:
      printf( "Turning GPS off.\n" );
      error = gps_power_set( false );
      assert( !error );
      
      // 10 seconds off
      timeout.time = system_timeMs_get() + 10000;
      system_timeout_register( &timeout );
      break;
      
    case STATE_RECEIVING_POSITIONS:
      timeout.time = system_timeMs_get() + 10000;
      system_timeout_register( &timeout );
      break;
      
    default:
      break;
  }
}


static void gpsReadyCallback( Error error )
{
  printf( "GPS is ready\n" );
  
  ready = true;
  state = STATE_READY;
  
  gps_getPositions( positionCallback );
  
}

static void positionCallback( const sGPSPosition *position )
{
  if( state == STATE_READY )
    stateNext( STATE_RECEIVING_POSITIONS );

  printf( "Long: %f lat: %f, accuracy: %f m\n",
         ((double)position->longitude) / 1E7,
         ((double)position->latitude) / 1E7,
         ((double) position->accuracy) / 1000 );
}
