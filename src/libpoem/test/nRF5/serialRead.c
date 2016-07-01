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
#include <poem/logging.h>
#include <poem/system.h>
#include <poem/nRF5/serial_app_nRF5.h>
#include <poem/nRF5/system_nRF5.h>

#define UART_RX_PIN 5
#define UART_TX_PIN 6

static bool done;

/*
 * static functions
 */

/*
 * Global variables
 */

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

  log_printf( "Initialized, starting loop\n" );

  done = false;

  while( !done )
  {
    char ch;
    int length;

    do
    {
      length = 1;
      error = serial_read( NULL, (uint8_t *) &ch, &length );
      if( error != NULL )
	log_printf( "Error!\n" );
      else
	if( length == 1 )
	  log_printf( "%c", ch );
	else
	  assert( length == 0 );
    } while( length > 0 );

    system_waitForEvent();
  }
}

