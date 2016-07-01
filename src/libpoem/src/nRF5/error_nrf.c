/*
   Erl's embedded library

   Nordic Radio NRf SDK 11 error handling
*/

#include <stdlib.h>
#include "sdk_errors.h" // for ret_error_t

#include "poem/error.h"
#include "poem/nRF5/error_nRF5.h"

static int errModule = -1;

void nRF5_error_init( void )
{
  if( errModule == -1 )
    errModule = err_registerModule();
}

void nRF5_makeError( Error error, ret_code_t retCode )
{
  error->module = errModule;
  error->error = (int)retCode;
  error->cause = NULL;
  error->moduleName = "nRF";

  switch( retCode )
  {
    case NRF_ERROR_INVALID_PARAM:
      error->description = "Invalid Parameter";
      break;

    default:
      error->description = "<Unknown>";
      break;
  }
}
