/*
  Erl's embedded library error structure
*/

#include "poem/error.h"

static int errModuleCount;

void err_init()
{
  errModuleCount = 0;
}

int err_registerModule()
{
  errModuleCount++;  // Module #0 not uesd

  return errModuleCount;
}
