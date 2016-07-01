/*
 * serial_nrf51822.c
 * 
 * SDK 11 
 */

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "sdk_errors.h"
#include "nrf_drv_uart.h"

#include <poem/circularBuffer.h>
#include <poem/error.h>
#include <poem/serial.h>
#include <poem/system.h> // for system_setEventPending
#include <poem/logging.h>

#include <poem/nRF5/error_nRF5.h>

#define READ_COUNT 1

/*
 *   static functions
 */
static void serialEventHandler( nrf_drv_uart_event_t *event, void *context );

/*
 *  static data
 */
static sCircularBuffer txCircularBuffer;
static uint8_t txBuffer[ 256 ];
static uint8_t byteToWrite;

/* 
   rxBuffer will contain received data from rxHead to rxTail - 1 mod rxBuffer
   Number of bytes is rxTail - rxBuffer (mod rxBuffer size)
*/
static sCircularBuffer rxCircularBuffer;
static uint8_t rxBuffer[ 1024 ];
static uint8_t rxBytes[READ_COUNT];

// static uint8_t byteRead
static sError error;
static int errorModule = -1;

// todo: pass rxBuffer and txBuffer and their sizes as parameters,
// so app can decide how big they should be
Error nrf51822_app_serial_init( SerialChannel channel,
				uint32_t baudRate,
				uint8_t interruptPriority, 
				int txdPin, int rxdPin,
				int ctsPin, int rtsPin )
{
  ret_code_t retCode;
  nrf_drv_uart_config_t config;
  // Error subError;

  if( errorModule == -1 )
    errorModule = err_registerModule();

  config.pseltxd = txdPin;
  config.pselrxd = rxdPin;
  config.pselcts = ctsPin;
  config.pselrts = rtsPin;
  config.p_context = channel; //?
  config.hwfc = ((ctsPin < 0) || (rtsPin < 0)) ? NRF_UART_HWFC_DISABLED : NRF_UART_HWFC_ENABLED;
  config.parity = NRF_UART_PARITY_EXCLUDED;
  
  switch( baudRate )
  {
    case 1200:
      config.baudrate = NRF_UART_BAUDRATE_1200;
      break;

    case 2400:
      config.baudrate = NRF_UART_BAUDRATE_2400;
      break;

    case 4800:
      config.baudrate = NRF_UART_BAUDRATE_4800;
      break;

    case 9600:
      config.baudrate = NRF_UART_BAUDRATE_9600;
      break;

    case 14400:
      config.baudrate = NRF_UART_BAUDRATE_14400;
      break;

    case 19200:
      config.baudrate = NRF_UART_BAUDRATE_19200;
      break;

    case 28800:
      config.baudrate = NRF_UART_BAUDRATE_28800;
      break;

    case 57600:
      config.baudrate = NRF_UART_BAUDRATE_57600;
      break;

    case 76800:
      config.baudrate = NRF_UART_BAUDRATE_76800;
      break;

    case 115200:
      config.baudrate = NRF_UART_BAUDRATE_115200;
      break;

    case 230400:
      config.baudrate = NRF_UART_BAUDRATE_230400;
      break;

    case 250000:
      config.baudrate = NRF_UART_BAUDRATE_250000;
      break;

    case 460800:
      config.baudrate = NRF_UART_BAUDRATE_460800;
      break;

    case 921600:
      config.baudrate = NRF_UART_BAUDRATE_921600;
      break;

    case 1000000:
      config.baudrate = NRF_UART_BAUDRATE_1000000;
      break;

    default:
      error.module = errorModule;
      error.error = SERIAL_ERR_INVALID_BAUDRATE;
      error.cause = NULL;
      error.moduleName = "nRF Serial";
      error.description = "Invalid baud rate";

      return &error;
  }

  config.interrupt_priority = interruptPriority;

  circBuf_create( sizeof( txBuffer), &txCircularBuffer, txBuffer );
  circBuf_create( sizeof( rxBuffer), &rxCircularBuffer, rxBuffer );
  
  retCode = nrf_drv_uart_init( &config, serialEventHandler );
  if( retCode != NRF_SUCCESS )
  {
    nRF5_makeError( &error, retCode );
    return &error;
  }

  nrf_drv_uart_rx_enable();

  retCode = nrf_drv_uart_rx( rxBytes, READ_COUNT );
  if( retCode != NRF_SUCCESS )
  {
    nRF5_makeError( &error, retCode );

    // disable rx? uninit serial?
    return &error;
  }

  return NULL;
}

static void serialEventHandler( nrf_drv_uart_event_t *event, void *context )
{
  ret_code_t retCode;
  // bool tempBool;

  switch( event->type )
  {
    case NRF_DRV_UART_EVT_TX_DONE:
      // write the next character from the buffer
      if( circBuf_getElementCount( &txCircularBuffer ) > 0 )
      {
        byteToWrite = circBuf_popTailByte( &txCircularBuffer );
	// this line makes all the difference
	// log_printf( "w %2xd\n", byteToWrite );
	log_printf( "%d", byteToWrite );
	// try swapping buffers?
	// must clear tx event before writing new byte?
        retCode = nrf_drv_uart_tx( &byteToWrite, 1 );
	assert( retCode == NRF_SUCCESS );
      }

      system_setEventPending();
      break;

    case NRF_DRV_UART_EVT_RX_DONE:
      // strange, I have seen bytes == 0 here
      // if( event->data.rxtx.bytes > 0 )
      {
	int i;
      // assert( event->data.rxtx.bytes == 1 );
	// could oveflow. No handling for that now.
	// assert( byte == event->data.rxtx.p_data[0] );
	// byte and p_data[0] are not equal. Not sure which to use. 
        for( i = 0; i < event->data.rxtx.bytes; i++ )
	/* tempBool = */ circBuf_pushByte( &rxCircularBuffer, event->data.rxtx.p_data[i] );

	// ignore dropped data
	// assert( tempBool );

	retCode = nrf_drv_uart_rx( rxBytes, READ_COUNT );
	assert( retCode == NRF_SUCCESS );

	system_setEventPending();
	// 
      }
      break;

     case NRF_DRV_UART_EVT_ERROR:
       // if error mask is 1, then we have a receive overrun. Ignore it.
       
       // According to this post, we must re-enable the UART interrupt:
       // https://devzone.nordicsemi.com/question/79061/nrf_drv_uart_evt_error-issue/
       nrf_uart_int_enable(NRF_UART0, NRF_UART_INT_MASK_RXDRDY | NRF_UART_INT_MASK_ERROR);

       system_setEventPending(); // not sure if we need this
       break;

    default:
      log_printf( "serial_nRF5_app.c: strange event=%x\n", event->type );
      // log, ignore
      break;
  }
}


Error serial_write( SerialChannel channel, const uint8_t *buffer, int *length )
{
  ret_code_t retCode;
  int oldTxBufferLength;
  int i;

  oldTxBufferLength = circBuf_getElementCount( &txCircularBuffer );
  
  for( i = 0; (i < *length) && circBuf_pushByte( &txCircularBuffer, buffer[i]); i++ )
    ;
  
  *length = i;
  
  if( oldTxBufferLength == 0 )
  {
    // start UART write
    byteToWrite = circBuf_popTailByte( &txCircularBuffer );
    retCode = nrf_drv_uart_tx( &byteToWrite, 1 );

    if( retCode != NRF_SUCCESS )
    {
      nRF5_makeError( &error, retCode );
      
      return &error;
    }
  }

  return NULL;
}

Error serial_read( SerialChannel channel, uint8_t *buffer, int *length )
{
  int i;
  
  for( i = 0; (circBuf_getElementCount( &rxCircularBuffer ) > 0) && (i < *length); i++ )
    buffer[i] = circBuf_popTailByte( &rxCircularBuffer );
  
  *length = i;
  
  return NULL;
}
