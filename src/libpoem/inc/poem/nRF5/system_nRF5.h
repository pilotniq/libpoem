//
// system_nrf5.h
//
#ifndef EMBLIB_NRF5_SYSTEM_H
#define EMBLIB_NRF5_SYSTEM_H

#include <poem/error.h>
#include <poem/nRF5/error_nRF5.h>

#include "nrf_sdm.h" // for nrf_clock_lf_cfg_t

#if NRF_SDK_VERSION_MAJOR <= 14
Error nRF5_system_init( int centralLinkCount, int peripheralLinkCount, nrf_clock_lf_cfg_t *lfClkCfg );
#else
Error nRF5_system_init( void );
#endif

#endif
