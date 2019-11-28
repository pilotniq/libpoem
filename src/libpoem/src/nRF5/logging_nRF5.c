/*
  nRF5 Logging

  Uses the experimental UART/RTT Logging library in nRF5 SDK11
*/

#include <stdarg.h>
#include <stdbool.h>

#include "SEGGER_RTT.h"

#include <poem/error.h>
#include <poem/logging.h>
#include <poem/nRF5/error_nRF5.h>

// Where is the documentation for the Segger API? 
// I can only find https://www.segger.com/jlink-rtt.html#RTT_Implementation 
// which has no details
#define LOG_TERMINAL_NORMAL         (0)

// static sError error;
#if 0
Error logging_init(void)
{
  uint32_t errCode;
  static bool initialized = false;

  nRF5_error_init();

  if (SEGGER_RTT_ConfigUpBuffer(LOG_TERMINAL_NORMAL,
                                  "Normal",
                                  buf_normal_up,
                                  BUFFER_SIZE_UP,
                                  SEGGER_RTT_MODE_NO_BLOCK_TRIM
                                 )
        != 0)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (SEGGER_RTT_ConfigDownBuffer(LOG_TERMINAL_INPUT,
                                   "Input",
                                   buf_down,
                                   BUFFER_SIZE_DOWN,
                                   SEGGER_RTT_MODE_NO_BLOCK_SKIP
                                  )
        != 0)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    initialized = true;

    return NRF_SUCCESS;

  errCode = NRF_LOG_INIT();

  if( errCode != NRF_SUCCESS )
  {
    nRF5_makeError( &error, errCode );
    return &error;
  }

  return NULL;
}
#endif

// crazyness, the Segger library includes this file, but it is not listed in any .h-files.

int SEGGER_RTT_vprintf(unsigned BufferIndex, const char * sFormat, va_list * pParamList);

void poem_log_printf( const char *format, ... )
{
    va_list p_args;
    va_start(p_args, format);

    SEGGER_RTT_vprintf( LOG_TERMINAL_NORMAL, format, &p_args);

    va_end(p_args);
}

