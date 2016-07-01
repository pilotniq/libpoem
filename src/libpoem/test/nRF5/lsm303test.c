/*
  Test program for LSM303DLHC accellerometer / magnetometer
*/

#include <assert.h>

#include <poem/gps.h>
#include <poem/error.h>
#include <poem/logging.h>
#include <poem/serial.h>
#include <poem/system.h>

#include <poem/lsm303dlhc.h>
#include <poem/system.h>

#include <poem/nRF5/i2c_app_nRF5.h>
#include <poem/nRF5/system_nRF5.h>

#include "custom_board.h" // for NRF_CLOCK_LFCLKSRC

#define I2C_SCL_PIN 1
#define I2C_SDA_PIN 0

int main( int argc, char **argv )
{
  Error error;
  nrf_clock_lf_cfg_t lfClk = NRF_CLOCK_LFCLKSRC;
  sI2Cchannel i2cChannel;
  int16_t acc[3];
  int16_t mag[3];
  uint16_t heading;
  bool done;

  error = nRF5_system_init( 0, 0, &lfClk );
  if( error != NULL )
  {
    log_printf( "nRF5_system_init failed: %s (%d) %s\n", error->moduleName, error->error, error->description );
    return -1;
  }

  log_printf( "libPoem nRF51822 test of LSM303 compass\n" );

  error = nrf51822_app_i2c_init( &i2cChannel, 400000, I2C_SCL_PIN, I2C_SDA_PIN );
  if( error != NULL )
  {
    log_printf( "i2c init failed: %s (%d) %s\n", error->moduleName, error->error, error->description );
    return -1;
  }

  lsm303dlhc_init( &i2cChannel );
  
  lsm303dlhc_acc_setDataRate( ACC_RATE_1_HZ );
  lsm303dlhc_mag_setMode( LSM303DLHC_MAG_MODE_CONTINUOUS );

  lsm303dlhc_acc_read( acc );
  lsm303dlhc_mag_read( mag );

  log_printf( "Acc: %d,%d,%d\n", acc[0], acc[1], acc[2] );
  log_printf( "Mag: %d,%d,%d\n", mag[0], mag[1], mag[2] );

  heading = lsm303dlhc_tilt_compensate( acc, mag );
  heading = ((uint32_t) heading) * 36000 / UINT16_MAX;

  log_printf( "Acc: %d,%d,%d\n", acc[0], acc[1], acc[2] );
  log_printf( "Mag: %d,%d,%d\n", mag[0], mag[1], mag[2] );
  log_printf( "Heading=%d\n", heading );

  // todo: set up a timer, twicea second query mag and acc
  done = false;
  while( !done )
  {
    system_waitForEvent();
  }
  
  return 0;
}
