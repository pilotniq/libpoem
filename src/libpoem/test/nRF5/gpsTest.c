//
//  gpsTest.c for nRF
//
//  Created by Erland Lewin on 2016-05-18.
//
//

#include <assert.h>

#include <poem/gps.h>
#include <poem/error.h>
#include <poem/logging.h>
#include <poem/serial.h>
#include <poem/system.h>

#include <poem/gps_ublox6.h>
#include <poem/system.h>

#include <poem/nRF5/serial_app_nRF5.h>
#include <poem/nRF5/system_nRF5.h>

#include "custom_board.h" // for NRF_CLOCK_LFCLKSRC

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
  // sSerialChannel serial;
  Error error;
  nrf_clock_lf_cfg_t lfClk = NRF_CLOCK_LFCLKSRC;

  ready = false;
  error = nRF5_system_init( 0, 0, &lfClk );
  if( error != NULL )
  {
    log_printf( "nRF5_system_init failed: %s (%d) %s\n", error->moduleName, error->error, error->description );
    return -1;
  }

  error = nrf51822_app_serial_init( NULL, 9600, 1, 6, 5, -1, -1 );
  if( error != NULL )
  {
    log_printf( "serial_init failed: %s (%d) %s\n", error->moduleName, error->error, error->description );
    return -1;
  }

  state = STATE_WAIT_FOR_READY;
  
  gps_ublox6_init( NULL /* &serial */, gpsReadyCallback );
  
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
        log_printf( "Turning on GPS\n");
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
      log_printf( "Turning GPS off.\n" );
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
  log_printf( "GPS is ready\n" );
  
  ready = true;
  state = STATE_READY;
  
  gps_getPositions( positionCallback );
  
}

static void positionCallback( const sGPSPosition *position )
{
  if( state == STATE_READY )
    stateNext( STATE_RECEIVING_POSITIONS );

  // printf doesn't support float?
#if 0
  log_printf( "Long: %d, lat: %d, accuracy: %d m\n",
	      ((double)position->longitude) / 1E7,
	      ((double)position->latitude) / 1E7,
	      ((double) position->accuracy) / 1000 );
#else
  log_printf( "Long: %d, lat: %d, accuracy: %d mm\n",
	      position->longitude,
	      position->latitude,
	      position->accuracy / 1000 );
#endif
}
