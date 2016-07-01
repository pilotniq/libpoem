/*
 *  Test of serial write on nRF51822
 */

#include <assert.h>
#include <stdbool.h>
#include <string.h>

// nRF includes 
#include "app_util_platform.h"
#include "custom_board.h"
#include "nrf_sdm.h" // for nrf_clock_lf_cfg_t

// poem includes
#include <poem/system.h>
#include <poem/nRF5/serial_app_nRF5.h>
#include <poem/nRF5/system_nRF5.h>

#define UART_RX_PIN 5
#define UART_TX_PIN 6

static bool done;
static int sendProcessedCount;

typedef enum {
  APP_STATE_HELLO,
  APP_STATE_BYE
} AppState;

/*
 * static functions
 */
static void appStateNext( AppState newState );
static void appStateControl( void );

/*
 * Global variables
 */
static AppState state;
static uint8_t *helloBuffer = (uint8_t *) "Hello!\r\n";
static uint8_t *byeBuffer = (uint8_t *) "Good bye.\r\n";

/*
 * Start of code
 */
int main( int argc, char **argv )
{
  Error error;
  nrf_clock_lf_cfg_t lfClk = NRF_CLOCK_LFCLKSRC;

  error = nRF5_system_init( 0, 0, &lfClk );
  assert( error == NULL );

  error = nrf51822_app_serial_init( NULL, 9600, APP_IRQ_PRIORITY_HIGH, UART_TX_PIN, UART_RX_PIN, -1, -1 );
  assert( error == NULL );

  done = false;
  state = APP_STATE_HELLO;

  while( !done )
  {
    appStateControl();
    system_waitForEvent();
  }
}

static void appStateControl( void )
{
  bool sendDone;
  Error error;

  switch( state )
  {
    case APP_STATE_HELLO:
      error = serial_writeFromBuffer( NULL, helloBuffer, strlen( (char *) helloBuffer ), &sendProcessedCount, &sendDone );
      assert( error == NULL );
	  
      if( sendDone )
	appStateNext( APP_STATE_BYE );
      break;

    case APP_STATE_BYE:
      error = serial_writeFromBuffer( NULL, byeBuffer, strlen( (char *) byeBuffer ), &sendProcessedCount, &sendDone );
      assert( error == NULL );
	  
      if( sendDone )
	appStateNext( APP_STATE_HELLO );
      break;

    default:
      assert( false );
  }
}

static void appStateNext( AppState newState )
{
  sendProcessedCount = 0;
  state = newState;

  appStateControl(); // if we don't write now, we will wait for event forever, maybe?
}
