#include <assert.h>
#include <stdlib.h>

#include "custom_board.h" // for NRF_CLOCK_LFCLKSRC
#include "nrf_sdm.h" // for nrf_clock_lf_cfg_t

#include <poem/error.h>
#include <poem/nRF5/system_nRF5.h>
#include <poem/logging.h>

int main( int argc, char **argv )
{
  Error error;
  nrf_clock_lf_cfg_t lfClk = NRF_CLOCK_LFCLKSRC;

  error = nRF5_system_init( 0, 0, &lfClk );
  assert( error == NULL );

  while( 1 )
  {
    log_printf( "Hello, %s\r\n", "Word" );
  }
}
