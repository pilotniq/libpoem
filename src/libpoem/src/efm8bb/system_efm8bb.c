//
//  system_nRF5.c
//
#define NDEBUG
#include <assert.h>
#include <stdint.h>

#include <SI_EFM8BB1_Register_Enums.h>

#include <poem/error.h>
#include <poem/efm8bb/system_efm8bb.h>

/*
 * start of code
 */
Error efm8bb_system_init( )
{
  // Disable watchdog
  WDTCN = 0xDE; //First key
  WDTCN = 0xAD; //Second key

  return NULL;
}

//-----------------------------------------------------------------------------
// SiLabs_Startup() Routine
// ----------------------------------------------------------------------------
// This function is called immediately after reset, before the initialization
// code is run in SILABS_STARTUP.A51 (which runs before main() ). This is a
// useful place to disable the watchdog timer, which is enable by default
// and may trigger before main() in some instances.
//-----------------------------------------------------------------------------
void SiLabs_Startup (void)
{
  // Call hardware initialization routine
  // enter_DefaultMode_from_RESET();
}

uint32_t system_timeMs_get(void)
{
	return 0; // NIY
}
