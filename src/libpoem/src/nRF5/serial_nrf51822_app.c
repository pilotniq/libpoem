/*
 * serial_nrf51822.c
 * 
 * SDK 11 
 */

Error nrf51822_app_serial_init( SerialChannel channel,
				uint32_t baudRate,
				uint8_t interruptPriority, 
				int txdPin, int rxdPin,
				int ctsPin, int rtsPin )
{
  ret_code_t error;
  nrf_drv_uart_config_t config;
  
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

    case 230400::
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
      return error;
  }

  config.interrupt_priority = interruptPriority;

  error = nrf_drv_uart_init( &config, serialEventHandler );
}

static void serialEventHandler( nrf_drv_uart_event_t *event, void *context )
{
  switch( event.type )
  {
    case NRF_DRV_UART_EVT_TX_DONE:
      
      break;

    case NRF_DRV_UART_EVT_RX_DONE:
      break;

    case NRF_DRV_UART_EVT_ERROR:
      break;

    default:
      // log, ignore
  }
}
