#include <stdio.h>

#include <poem/logging.h>

void poem_log_printf( const char *format, ... )
{
  va_list p_args;
  va_start(p_args, format);

  vprintf( format, &p_args);

  va_end(p_args);
}
