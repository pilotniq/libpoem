/*
  Erl's embedded library

  unix.h - for implementations on systems running Unix
*/

#ifndef EMBLIB_NRF5
#define EMBLIB_NRF5

#include "sdk_errors.h" // for ret_error_t

#include <poem/error.h>

void nRF5_error_init( void );
void nRF5_makeError( Error error, ret_code_t retCode );

#endif
