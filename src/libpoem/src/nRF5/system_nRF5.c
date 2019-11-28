//
//  system_nRF5.c
//
#include <assert.h>

#include "nrf_error.h" // for NRF_SUCCESS
#include "nrf_soc.h" // for sd_app_evt_wait
#include "nrf_delay.h"

#if NRF_SDK_VERSION_MAJOR <= 14
#include "softdevice_handler.h" // for SOFTDEVICE_HANDLER_INIT
#include "app_ram_base.h" // for APP_RAM_BASE_CENTRAL_LINKS_...
#else
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#endif

#include "app_timer.h"    // for APP_TIMER...

#include <poem/error.h>
#include <poem/nRF5/system_nRF5.h>
#include <poem/nRF5/error_nRF5.h>

#if NRF_SDK_VERSION_MAJOR <= 14
static Error ble_stack_init( int centralLinkCount, int peripheralLinkCount, 
			                       nrf_clock_lf_cfg_t *lfClkCfg );
#else
static Error ble_stack_init( );
#endif

static void systemTimerTimeoutHandler( void *context );

#if NRF_SDK_VERSION_MAJOR <= 14
static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
#endif

/*
 * static variables
 */
static sError myError;
static uint32_t tickCount4kHz;
static bool eventPending;

APP_TIMER_DEF( systemTimer );

/*
 * start of code
 */
#if NRF_SDK_VERSION_MAJOR <= 14
Error nRF5_system_init( int centralLinkCount, int peripheralLinkCount, nrf_clock_lf_cfg_t *lfClkCfg )
#else
Error nRF5_system_init( )
#endif
{
  Error error;
  uint32_t retCode;

  nRF5_error_init();

  // initialize the app timer library
  // we want ms resolution for the system clock. Exact precision is impossible since clock 32768 Hz divided by prescaler.
  // say we want more than 2 kHz resolution, we would want a prescaler of 16 to get 2048 Hz
  // At 32 768 Hz, a 32 bit counter will overflow in 2^32 / 2^15 = 2^(32-15) = 2^17 = 36 hours. Not great, fix later.
#if NRF_SDK_VERSION_MAJOR < 13
  APP_TIMER_INIT( 16, 1 /* OP_QUEUE_SIZE */, NULL /* scheduler_function */);
#else
  retCode = app_timer_init();
  assert( retCode == NRF_SUCCESS );
#endif
  // set up a timer to be called to update the system time
  retCode = app_timer_create( &systemTimer, APP_TIMER_MODE_REPEATED, systemTimerTimeoutHandler );
  assert( retCode == NRF_SUCCESS );

#if NRF_SDK_VERSION_MAJOR <= 14
  error = ble_stack_init( centralLinkCount, peripheralLinkCount, lfClkCfg );
#else
  error = ble_stack_init( );
#endif
  if( error != NULL )
    goto FAIL;

  eventPending = false;

  // apparently, no less than 5 ticks are allowed, so use 8.
  retCode = app_timer_start( systemTimer, 8, NULL );
  if( retCode != NRF_SUCCESS )
  {
    nRF5_makeError( &myError, retCode );
    error = &myError;

    goto FAIL;
  }

  return NULL;

 FAIL:
  return error;
}

void system_waitForEvent( void )
{
  uint32_t err_code;

  if( eventPending )
  {
    eventPending = false;
    return;
  }

  err_code = sd_app_evt_wait();

  assert( err_code == NRF_SUCCESS );
}

void system_setEventPending( void )
{
  eventPending = true;
}

#if NRF_SDK_VERSION_MAJOR <= 14
static Error ble_stack_init( int centralLinkCount, int peripheralLinkCount, nrf_clock_lf_cfg_t *lfClkCfg )
#else
static Error ble_stack_init( )
#endif
{
  uint32_t err_code;
  // nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

  // Initialize the SoftDevice handler module.                                                                                   

#if NRF_SDK_VERSION_MAJOR <= 14
  ble_enable_params_t ble_enable_params;
  uint32_t app_ram_start_addr;


  SOFTDEVICE_HANDLER_INIT( lfClkCfg, NULL);

  err_code = softdevice_enable_get_default_config( centralLinkCount, 
						   peripheralLinkCount,
						   &ble_enable_params);
  if( err_code != NRF_SUCCESS )
    goto FAIL;
  
  //Check the ram settings against the used number of links
  // CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
  switch( centralLinkCount )
  {
    case 0:
      switch( peripheralLinkCount )
      {
        case 0:
	  app_ram_start_addr = APP_RAM_BASE_CENTRAL_LINKS_0_PERIPH_LINKS_0_SEC_COUNT_0_MID_BW;
	  break;

        case 1:
	  app_ram_start_addr = APP_RAM_BASE_CENTRAL_LINKS_0_PERIPH_LINKS_1_SEC_COUNT_0_MID_BW;
	  break;

        default:
	  assert( false );
      }
      break;

    case 1:
      switch( peripheralLinkCount )
      {
        case 0:
	  app_ram_start_addr = APP_RAM_BASE_CENTRAL_LINKS_1_PERIPH_LINKS_0_SEC_COUNT_0_MID_BW;
	  break;

        case 1:
	  app_ram_start_addr = APP_RAM_BASE_CENTRAL_LINKS_1_PERIPH_LINKS_1_SEC_COUNT_0_MID_BW;
	  break;
	  
        default:
	  assert( false );
	  break;
      }

    default:
      assert( false );
      break;
  }

  err_code = sd_check_ram_start(app_ram_start_addr);
  if( err_code != NRF_SUCCESS )
    goto FAIL;
  
  // Enable BLE stack.
  err_code = softdevice_enable(&ble_enable_params);
  if( err_code != NRF_SUCCESS )
    goto FAIL;

#if 1  // NIY 
  // Register with the SoftDevice handler module for BLE events.
  err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
  if( err_code != NRF_SUCCESS )
    goto FAIL;
#endif

#else
  nrf_sdh_enable_request();

  uint32_t ram_start;
  nrf_sdh_ble_app_ram_start_get(&ram_start);
  /* Configure the stack using the SoftDevice API (@ref sd_ble_cfg_set). */
  nrf_sdh_ble_enable(&ram_start);             /* Enable the BLE stack. */
#endif

  return NULL;

 FAIL:
  nRF5_makeError( &myError, err_code );
  return &myError;
}

#if NRF_SDK_VERSION_MAJOR < 14
/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *          been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
  asm( "nop" );
#if 0
  printf( "ble_evt_dispatch\r\n" );

  // dm_ble_evt_handler(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_lns_c_on_ble_evt(&m_ble_lns_c,p_ble_evt);
    on_ble_evt(p_ble_evt);
#endif
}
#endif
// 32768 = 1s 
// 32.768 = 1 ms
static void systemTimerTimeoutHandler( void *context )
{
  // We are now being called every 8 ticks, clock is 32768 / 8 = 4096 Hz
  tickCount4kHz++;
}

uint32_t system_timeMs_get(void)
{
  // Apparently, for easy use of the nRF libraries, it requires that the Timer App Library be used
  // see https://devzone.nordicsemi.com/question/938/rtc0-rtc1-and-the-app_timer-with-softdevice/
  
  // we are using 8 ticks 
  // return (( (uint64_t) tickCount4kHz) * 1000) / 4096; // never mind the overflow, we're just interested in the finer resolution
  // 
  // return (( (uint64_t) tickCount4kHz) * 125) / 512; // never mind the overflow, we're just interested in the finer resolution
  return (( (uint64_t) tickCount4kHz) * 125) / 128; // For unknown reasons, need 4x the ticks calculated above
}
#if 0
static void lns_c_evt_handler(ble_lns_c_t * p_ble_lns_c, const ble_lns_c_evt_t * p_ble_lns_evt)
{
  asm( "nop" );
}
#endif

void system_delayMs( int ms )
{
  nrf_delay_ms( ms );
}
