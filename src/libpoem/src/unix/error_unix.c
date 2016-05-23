/*
   Erl's embedded library
*/

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "poem/error.h"

static int errModule = -1;

void unix_init( void )
{
  if( errModule == -1 )
    errModule = err_registerModule();
}

void unix_makeError( Error error )
{
  error->module = errModule;
  error->error = errno;
  error->cause = NULL;
  error->moduleName = "Unix";
  error->description = strerror( errno );
}
