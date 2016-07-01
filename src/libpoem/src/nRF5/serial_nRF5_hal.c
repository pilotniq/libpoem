/*
 * serial_nrf51822.c
 * 
 * SDK 11 
 */

#include <stdint.h>
#include <stdbool.h>

#include "nrf_uart.h"

static bool txBusy;
static uint8_t txBuffer[ 256 ];

/* 
   rxBuffer will contain received data from rxHead to rxTail - 1 mod rxBuffer
   Number of bytes is rxTail - rxBuffer (mod rxBuffer size)
*/
static uint8_t rxBuffer[ 256 ];
static int rxHead;
static int rxTail;
static sError error;
static int errorModule = -1;

Error nrf51822_app_serial_init( SerialChannel channel,
				uint32_t baudRate,
				uint8_t interruptPriority, 
				int txdPin, int rxdPin,
				int ctsPin, int rtsPin )
{
  ret_code_t retCode;
  nrf_drv_uart_config_t config;
  nrf_uart_baudrate_t baudRateCode;
  Error subError;
  bool hwFC;

  if( errModule == -1 )
    errModule = err_registerModule();

  hwFC = (ctsPin >= 0) && (rtsPin >= 0);

  /* Setup the pins */
  // some code from nrf_drv_uart.c
  nrf_gpio_pin_set(p_config->pseltxd);
  nrf_gpio_cfg_output(p_config->pseltxd);
  nrf_gpio_cfg_input(p_config->pselrxd, NRF_GPIO_PIN_NOPULL);

    /*
  config.pseltxd = txdPin;
  config.pselrxd = rxdPin;
  config.pselcts = ctsPin;
  config.pselrts = rtsPin;
  config.p_context = channel; //?
  config.hwfc = ((ctsPin < 0) || (rtsPin < 0)) ? NRF_UART_HWFC_DISABLED : NRF_UART_HWFC_ENABLED;
  config.parity = NRF_UART_PARITY_EXCLUDED;
    */
  switch( baudRate )
  {
    case 1200:
      baudRateCode = NRF_UART_BAUDRATE_1200;
      break;

    case 2400:
      baudRateCode = NRF_UART_BAUDRATE_2400;
      break;

    case 4800:
      baudRateCode = NRF_UART_BAUDRATE_4800;
      break;

    case 9600:
      baudRateCode = NRF_UART_BAUDRATE_9600;
      break;

    case 14400:
      baudRateCode = NRF_UART_BAUDRATE_14400;
      break;

    case 19200:
      baudRateCode = NRF_UART_BAUDRATE_19200;
      break;

    case 28800:
      baudRateCode = NRF_UART_BAUDRATE_28800;
      break;

    case 57600:
      baudRateCode = NRF_UART_BAUDRATE_57600;
      break;

    case 76800:
      baudRateCode = NRF_UART_BAUDRATE_76800;
      break;

    case 115200:
      baudRateCode = NRF_UART_BAUDRATE_115200;
      break;

    case 230400::
      baudRateCode = NRF_UART_BAUDRATE_230400;
      break;

    case 250000:
      baudRateCode = NRF_UART_BAUDRATE_250000;
      break;

    case 460800:
      baudRateCode = NRF_UART_BAUDRATE_460800;
      break;

    case 921600:
      baudRateCode = NRF_UART_BAUDRATE_921600;
      break;

    case 1000000:
      baudRateCode = NRF_UART_BAUDRATE_1000000;
      break;

    default:
      error.module = errModule;
      error.error = NRF_ERR_SERIAL_INVALID_BAUDRATE;
      error.cause = NULL;
      error.moduleName = "nRF Serial";
      error.description = "Invalid baud rate";

      return &error;
  }

  nrf_uart_baudrate_set( NRF_UART0, baudRateCode );
  nrf_uart_configure( NRF_UART0, NRF_UART_PARITY_EXCLUDED, 
		      hwFC ? NRF_UART_HWFC_ENABLED : NRF_UART_HWFC_DISABLED );
  nrf_uart_txrx_pins_set( NRF_UART0, txdPin, rxdPin );
  if( hwFC )
  {
    nrf_gpio_cfg_input( ctsPin, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_pin_set( rtsPin );
    nrf_gpio_cfg_output( rtsPin );
    nrf_uart_hwfc_pins_set(NRF_UART0, rtsPin, ctsPin );
  }
 
  nrf_uart_enable( &NRF_UART0 );

  config.interrupt_priority = interruptPriority;

  retCode = nrf_drv_uart_init( &config, serialEventHandler );
  if( retCode != NRF_SUCCESS )
  {
    nrf_makeError( &error, retCode );
    return &error;
  }

  nrf_drv_uart_rx_enable();
  rxHead = 0; rxTail = 0;

  error = nrf_drv_uart_rx( rxBuffer, 1 );
  if( retCode != NRF_SUCCESS )
  {
    nrf_makeError( &error, retCode );

    // disable rx? uninit serial?
    return &error;
  }

  txBusy = false;

  return NULL;
}

static void serialEventHandler( nrf_drv_uart_event_t *event, void *context )
{
  switch( event.type )
  {
    case NRF_DRV_UART_EVT_TX_DONE:
      txBusy = false;
      break;

    case NRF_DRV_UART_EVT_RX_DONE:
      assert( event->data.rxtx.bytes == 1 );
      rxTail++;
      if( rxTail >= sizeof( rxBuffer ) )
	rxTail = 0;

      if( rxTail 
      break;

    case NRF_DRV_UART_EVT_ERROR:
      break;

    default:
      // log, ignore
  }
}

Error serial_write( SerialChannel channel, const uint8_t *buffer, int *length )
{
  ret_code_t retCode;
  int actualLength;

  if( isBusy )
  {
    *length = 0;
    return NULL;
  }

  // copy from buffer to my buffer
  if( *length > sizeof( txBuffer ) )
    *length = sizeof( txBuffer );

  memcpy( txBuffer, buffer, *length );

  isBusy = true;
  retCode = nrf_drv_uart_tx( buffer, *length );

  if( retCode != NRF_SUCCESS )
  {
    isBusy = false;
    nrf_makeError( &error, retCode );

    return &error;
  }
  else
    return NULL;
}

Error serial_read( SerialChannel channel, uint8_t *buffer, int *length )
{

}

void UART0_IRQHandler(void)
{
  if (nrf_uart_int_enable_check(NRF_UART0, NRF_UART_INT_MASK_ERROR) &&
      nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_ERROR))
  {
    nrf_drv_uart_event_t event;

    nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_ERROR);
    nrf_uart_int_disable(NRF_UART0, NRF_UART_INT_MASK_RXDRDY | NRF_UART_INT_MASK_ERROR);
    if (!m_cb.rx_enabled)
    {
	nrf_uart_task_trigger(NRF_UART0, NRF_UART_TASK_STOPRX);
    }
    event.type                   = NRF_DRV_UART_EVT_ERROR;
    event.data.error.error_mask  = nrf_uart_errorsrc_get_and_clear(NRF_UART0);
    event.data.error.rxtx.bytes  = m_cb.rx_buffer_length;
    event.data.error.rxtx.p_data = m_cb.p_rx_buffer;
    
    //abort transfer
    m_cb.rx_buffer_length = 0;
    m_cb.rx_secondary_buffer_length = 0;
    
    m_cb.handler(&event,m_cb.p_context);
  }
  else if (nrf_uart_int_enable_check(NRF_UART0, NRF_UART_INT_MASK_RXDRDY) &&
	   nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_RXDRDY))
    {
      rx_byte();
      if (m_cb.rx_buffer_length == m_cb.rx_counter)
        {
	  if (m_cb.rx_secondary_buffer_length)
            {
	      uint8_t * p_data     = m_cb.p_rx_buffer;
	      uint8_t   rx_counter = m_cb.rx_counter;
              
	      //Switch to secondary buffer.
	      m_cb.rx_buffer_length = m_cb.rx_secondary_buffer_length;
	      m_cb.p_rx_buffer = m_cb.p_rx_secondary_buffer;
	      m_cb.rx_secondary_buffer_length = 0;
	      m_cb.rx_counter = 0;
	      rx_done_event(rx_counter, p_data);
            }
	  else
            {
	      if (!m_cb.rx_enabled)
                {
		  nrf_uart_task_trigger(NRF_UART0, NRF_UART_TASK_STOPRX);
                }
                nrf_uart_int_disable(NRF_UART0, NRF_UART_INT_MASK_RXDRDY | NRF_UART_INT_MASK_ERROR);
                m_cb.rx_buffer_length = 0;
                rx_done_event(m_cb.rx_counter, m_cb.p_rx_buffer);
            }
        }
    }
  
  if (nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_TXDRDY))
  {
      if (m_cb.tx_counter < (uint16_t) m_cb.tx_buffer_length)
      {
	tx_byte();
      }
      else
      {
	nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_TXDRDY);
	if (m_cb.tx_buffer_length)
	{
	  tx_done_event(m_cb.tx_buffer_length);
	}
      }
    }

  if (nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_RXTO))
    {
        nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_RXTO);

        // RXTO event may be triggered as a result of abort call. In th
        if (m_cb.rx_enabled)
        {
            nrf_uart_task_trigger(NRF_UART0, NRF_UART_TASK_STARTRX);
        }
        if (m_cb.rx_buffer_length)
        {
            m_cb.rx_buffer_length = 0;
            rx_done_event(m_cb.rx_counter, m_cb.p_rx_buffer);
        }
    }
}
