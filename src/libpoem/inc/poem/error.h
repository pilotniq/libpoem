/* 
  Erl's embedded library error structure
*/

#ifndef EMBLIB_ERROR
#define EMBLIB_ERROR

typedef struct sError
{
  int module;
  int error;
  const char *description;
  const char *moduleName;
  struct sError *cause;
} sError, *Error;

void err_init();
int err_registerModule();

#endif
