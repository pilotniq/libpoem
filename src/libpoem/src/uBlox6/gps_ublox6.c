/*
   Portable Embedded Library (poem)

   gps_ublox6.c
*/

#include <assert.h>
#include <stdbool.h>
#include <stdio.h> // for debugging
#include <string.h> // for memcpy

#include <poem/logging.h>
#include <poem/serial.h>
#include <poem/gps.h>
#include <poem/gps_ublox6.h>
#include <poem/system.h>

// not sure about this
#define MAX_UBLOX_PACKET_LENGTH 256

/*
 * types
 */
typedef enum
{
  UBLOX_ACK_ACK = 0x01,
  
  UBLOX_CFG_PRT = 0x00,
  UBLOX_CFG_MSG = 0x01,
  UBLOX_CFG_RXM = 0x11,
  
  UBLOX_MON_VER = 0x04,
  
  UBLOX_NAV_POSLLH = 0x02,
  UBLOX_NAV_SOL    = 0x06,
  
  UBLOX_RXM_PMREQ = 0x41,
} UBloxID;

typedef enum
{
  UBLOX_CLASS_NAV = 0x01,
  UBLOX_CLASS_RXM = 0x02,
  UBLOX_CLASS_INF = 0x04,
  UBLOX_CLASS_ACK = 0x05,
  UBLOX_CLASS_CFG = 0x06,
  UBLOX_CLASS_MON = 0x0a,
  UBLOX_CLASS_AID = 0x0b,
  UBLOX_CLASS_TIM = 0x0d,
  UBLOX_CLASS_ESF = 0x10
} UBloxClass;

typedef enum
{
  UBLOX_FIX_NONE                  = 0x00,
  UBLOX_FIX_DEAD_RECKONING        = 0x01,
  UBLIX_FIX_2D                    = 0x02,
  UBLOX_FIX_3D                    = 0x03,
  UBLOX_FIX_3D_AND_DEAD_RECKONING = 0x04,
  UBLOX_FIX_TIME_ONLY             = 0x05
} GPSFixType;
/*
 * Initialization:
 *   -set right baud rate (not yet)
 *   -poll version to ensure we have a GPS
 *   -disable NMEA sentences
 *   -enable UBLOX protocol
 */
typedef enum
{
  STATE_NMEA_SET_PROTOCOLS_WRITE,
  STATE_POLLING_VERSION_SENDING,
  STATE_POLLING_VERSION_WAITING,
  STATE_DISABLE_NMEA_WRITE,
  STATE_DISABLE_NMEA_WAIT_ACK,
  STATE_LOCK_WAIT_WRITE,
  STATE_LOCK_WAIT_ACK,
  STATE_LOCK_WAIT,
  STATE_SET_PSM_WRITE,
  STATE_SET_PSM_WAIT_ACK,
  STATE_AFTER_PSM_WAIT,  // wait one second
  STATE_READY,
  STATE_SENDING_UBLOX,
  STATE_SENDING_UBLOX_WAKEUP,
  STATE_SENDING_UBLOX_PREWAIT,
  STATE_SENDING_UBLOX_SENDING,
  STATE_SENDING_UBLOX_WAIT_ACK,
  STATE_OFF
} State;

const char *stateNames[] = { "NMEA Set Protocols Write", 
			     "Polling Version Sending", 
			     "Polling Version waiting",
			     "Disable NMEA Write",
  "DISABLE_NMEA_WAIT_ACK",
  "LOCK_WAIT_WRITE",
  "LOCK_WAIT_ACK",
  "LOCK_WAIT",
  "SET_PSM_WRITE",
  "SET_PSM_WAIT_ACK",
  "AFTER_PSM_WAIT",  // wait one second
  "READY",
  "SENDING_UBLOX",
  "SENDING_UBLOX_WAKEUP",
  "SENDING_UBLOX_PREWAIT",
  "SENDING_UBLOX_SENDING",
  "SENDING_UBLOX_WAIT_ACK",
  "OFF"
};

/*
 * static function prototypes
 */
static void prepareUBloxSendMessage( uint8_t *buffer, int bufferSize, UBloxClass class, UBloxID id,
                                     int packetLength, const uint8_t *ptr );
// return whether to retry write
static bool nextState( State newState );
static void handleUBlox(void);
static void handleNMEA(void);
static int ublox_calcChecksum( const uint8_t *data, int length );
static void serviceWrite( void );
static void serviceRead( void );

/*
 * Constants
 */
// Line below sets NMEA and UBlox as allowed on input, only UBX on output, 9600 baud,
static const char *nmeaSetProtocols = "$PUBX,41,1,0003,0001,9600,0*16\r\n";

/*
 * File-global variables
 */
enum
{
  READSTATE_OUTSIDE,
  READSTATE_IN_NMEA,
  READSTATE_UBLOX_PREAMBLE_1,
  READSTATE_UBLOX_HEADER,
  READSTATE_UBLOX_PAYLOAD,
} readState;

static sError myError;

static bool inPowersave;
static int errorModule;
static State state;
static bool notifiedReady;
static int uBloxSendProcessedCount;
static int uBloxMessageLength;
static uint8_t uBloxSendBuffer[256];
static SerialChannel serial;
static GPSReadyCallback readyCallback;
static PositionCallback positionCallback;

static int ubloxPayloadLength;
static int readCursor;
// static int readTotal;
static uint8_t readBuffer[256];
static sTimeoutTime timeout;
// static bool blockingOnWrite;

/*
 * Start of code
 */
void gps_ublox6_init( SerialChannel _serial, GPSReadyCallback _readyCallback )
{
/* TODO
  error = serial_setSpeed( serial, 9600 );
  if( error != NULL )
    return error;
*/
  serial = _serial;
  readyCallback = _readyCallback;
  inPowersave = false;
  readState = READSTATE_OUTSIDE;

  errorModule = err_registerModule();
  notifiedReady = false;
  
  nextState( STATE_NMEA_SET_PROTOCOLS_WRITE );
  gps_service();
}

void gps_service( void )
{
  serviceRead();
  serviceWrite();
}

// Basically a state machine during initialization
// return true if blocking on write
static void serviceWrite( void )
{
  Error error;
  bool sendDone;
  uint8_t wakeUp = 0xff;
  int length;
  bool rewrite = false;
  
  do
  {
    // do writes
    switch( state )
    {
      case STATE_NMEA_SET_PROTOCOLS_WRITE:
        error = serial_writeFromBuffer( serial, (uint8_t *) nmeaSetProtocols,
                                       strlen(nmeaSetProtocols), &uBloxSendProcessedCount,
                                       &sendDone );
        if( sendDone )
          rewrite = nextState( STATE_POLLING_VERSION_SENDING );
        break;
        
      case STATE_POLLING_VERSION_SENDING:
	log_printf( "Polling version sending\r\n" );

        error = serial_writeFromBuffer( serial, uBloxSendBuffer, 8, &uBloxSendProcessedCount, &sendDone );
        assert( error == NULL );
        if( sendDone )
          rewrite = nextState( STATE_POLLING_VERSION_WAITING );
        break;
        
      case STATE_POLLING_VERSION_WAITING:
        if( system_timeout_expired( &timeout ) )
        {
          log_printf( "Polling version wait, timeout expired\n" );
          rewrite = nextState( STATE_POLLING_VERSION_SENDING );
        }
	else
	{
	  // uint32_t now;

	  // now = system_timeMs_get();
	  log_printf( "." ); // "Version Waiting, time=%d\n", now );
	}
        break; // don't send anything
        
      case STATE_DISABLE_NMEA_WRITE:
        error = serial_writeFromBuffer( serial, uBloxSendBuffer, 20+8, &uBloxSendProcessedCount, &sendDone );
        assert( error == NULL );

	log_printf( "Disable NMEA Write 2: processedCount=%d, sendDone=%d\n", uBloxSendProcessedCount, (int) sendDone );
        if( sendDone )
          rewrite = nextState( STATE_DISABLE_NMEA_WAIT_ACK );
        break;
        
      case STATE_DISABLE_NMEA_WAIT_ACK:
        if( system_timeout_expired( &timeout ) )
        {
          log_printf( "Disable NMEA Wait Ack, timeout expired\n" );
          rewrite = nextState( STATE_DISABLE_NMEA_WRITE );
        }
        break;
        
      case STATE_LOCK_WAIT_WRITE:
        uBloxMessageLength = 8 + 3;
        error = serial_writeFromBuffer( serial, uBloxSendBuffer, 3+8,
                                       &uBloxSendProcessedCount, &sendDone );
        assert( error == NULL );
        if( sendDone )
          rewrite = nextState( STATE_LOCK_WAIT_ACK );
        break;

      case STATE_SET_PSM_WRITE:
        error = serial_writeFromBuffer( serial, uBloxSendBuffer, 2+8, &uBloxSendProcessedCount, &sendDone );
        assert( error == NULL );
        if( sendDone )
          rewrite = nextState( STATE_SET_PSM_WAIT_ACK );
        break;
        
      case STATE_SET_PSM_WAIT_ACK:
        if( system_timeout_expired( &timeout ) )
        {
          log_printf( "Disable Set PSM Ack, timeout expired\n" );
          rewrite = nextState( STATE_SET_PSM_WRITE );
        }
        break;
        
      case STATE_AFTER_PSM_WAIT:
        if( system_timeout_expired( &timeout ) )
        {
          log_printf( "Waited one sec after PSM\n" );
          rewrite = nextState( STATE_READY );
        }
        break;
        
      case STATE_SENDING_UBLOX:
        if( inPowersave )
          rewrite = nextState( STATE_SENDING_UBLOX_WAKEUP );
        else
          rewrite = nextState( STATE_SENDING_UBLOX_SENDING );
        break;
        
      case STATE_SENDING_UBLOX_WAKEUP:
        length = 1;
        error = serial_write( serial, &wakeUp, &length );
        assert( error == NULL );
        
        if( length == 1 )
          rewrite = nextState( STATE_SENDING_UBLOX_PREWAIT );
        break;
        
      case STATE_SENDING_UBLOX_PREWAIT:
        if( system_timeout_expired( &timeout ) )
        {
          log_printf( "UBlox prewait expired.\n" );
          rewrite = nextState( STATE_SENDING_UBLOX_SENDING );
        }
        break;
        
      case STATE_SENDING_UBLOX_SENDING:
        error = serial_writeFromBuffer( serial, uBloxSendBuffer, uBloxMessageLength, &uBloxSendProcessedCount, &sendDone );
        if( sendDone )
        {
          if( uBloxSendBuffer[2] == UBLOX_CLASS_CFG )
            // wait for ack
            rewrite = nextState( STATE_SENDING_UBLOX_WAIT_ACK );
          else
            rewrite = nextState( STATE_READY );
        }
        break;
        
      case STATE_SENDING_UBLOX_WAIT_ACK:
        // TODO: Fix
        /*
         if( system_timeout_expired( &timeout ) )
         {
         log_printf( "Polling version wait, timeout expired\n" );
         nextState( STATE_DISABLE_NMEA_WRITE );
         }
         */
        break;
        
      default:
        // do no writes
        break;
    }
  } while( rewrite );
}

static void serviceRead( void )
{
  bool reread, done;
  int tempInt;
  Error error;
  
  do
  {
    reread = false;
    // read loop
    switch( readState )
    {
      case READSTATE_OUTSIDE:
        tempInt = 1;
        error = serial_read( serial, readBuffer, &tempInt );
        assert( error == NULL );
        
        if( tempInt == 1 )
        {
          switch( readBuffer[0] )
          {
            case 0xb5:
              readState = READSTATE_UBLOX_PREAMBLE_1;
              reread = true;
              break;
              
            case '$':
              readState = READSTATE_IN_NMEA;
              reread = true;
              readCursor = 1;
              break;
              
            default:
              log_printf( "Received junk character %02x\n", readBuffer[0]);
          }
        }
        break;
        
      case READSTATE_UBLOX_PREAMBLE_1:
        tempInt = 1;
        error = serial_read( serial, readBuffer + 1, &tempInt );
        assert( error == NULL );
        
        if( tempInt == 1 )
        {
          if( readBuffer[1] == 0x62 )
          {
            readCursor = 0;
            readState = READSTATE_UBLOX_HEADER;
          }
          else
            readState = READSTATE_OUTSIDE;
          
          reread = true;
        }
        break;
        
      case READSTATE_UBLOX_HEADER:
        // at least 6 bytes: class, id, length * 2, checksum * 2
        error = serial_readToBuffer( serial, readBuffer, 6, &readCursor, &done );
        assert( error == NULL );
        
        if( done )
        {
          ubloxPayloadLength = readBuffer[2] + (readBuffer[3] << 8);
          if( ubloxPayloadLength == 0 )
          {
            handleUBlox( );
            readState = READSTATE_OUTSIDE;
            
            reread = true;
          }
          else
          {
            if( ubloxPayloadLength > 300 )
              readState = READSTATE_OUTSIDE; // junk
            else
            {
              assert( ubloxPayloadLength < 300 );
              readCursor = 0;
              readState = READSTATE_UBLOX_PAYLOAD;

	      log_printf( "UBlox: Going head to payload read\r\n" );
            }
          }
          reread = true;
        }
        break;
        
      case READSTATE_UBLOX_PAYLOAD:
        assert( ubloxPayloadLength < 300 );

        error = serial_readToBuffer( serial, &(readBuffer[6]), ubloxPayloadLength, &readCursor, &done );
        if( done )
        {
          readCursor = 6 + readCursor;
          
          handleUBlox( );
          readState = READSTATE_OUTSIDE;
          
          reread = true;
        }
        break;
        
      case READSTATE_IN_NMEA:
        tempInt = 1;
        error = serial_read( serial, readBuffer + readCursor, &tempInt );
        if( tempInt == 1 )
        {
          readCursor++;
          
          if( readCursor > 82 ) // maximum NMEA sentence length
            readState = READSTATE_OUTSIDE; // abort this packet if too much data without terminator
          else if( (readBuffer[ readCursor - 1 ] == 0x0a) && (readBuffer[ readCursor - 2]== 0x0d))
          {
            handleNMEA( );
            readState = READSTATE_OUTSIDE;
          }
          else if( readBuffer[ readCursor - 1] & 0x80 )
          {
            // abort due to bad byte
            readState = READSTATE_OUTSIDE;
          }
          
          // possibly add timeout checking here
          reread = true;
        }
        break;
    }
  } while( reread );
}

static void handleUBlox()
{
  uint16_t checksum;
  UBloxClass class;
  UBloxID id;
  sGPSPosition position;
  
  // validate checksum
  checksum = ublox_calcChecksum( readBuffer, readCursor - 2 );
  
  if( readBuffer[ readCursor - 2 ] != (checksum & 0xff) ||
      readBuffer[ readCursor - 1 ] != ((checksum >> 8) & 0xff))
  {
    log_printf( "UBlox received packet has bad checksum\n" );
    
    return;
  }
  
  
  // packet will be in readBuffer, without the first two bytes (signature)
  class = (UBloxClass) readBuffer[0];
  id = (UBloxID) readBuffer[1];
  
  log_printf( "Got UBlox packet class %02x, id %02x\n", class, id );
  
  switch( class )
  {
    case UBLOX_CLASS_ACK:
      switch( id )
      {
        case UBLOX_ACK_ACK:
          switch( state )
          {
            case STATE_DISABLE_NMEA_WAIT_ACK:
              // unregister timeout
              system_timeout_unregister( &timeout );
              // fall-through
              
            case STATE_DISABLE_NMEA_WRITE:
              log_printf( "Got ack in STATE_DISABLE_NMEA_WRITE, Going to STATE_LOCK_WAIT_WRITE!\n" );
              nextState( STATE_LOCK_WAIT_WRITE );
              break;

            case STATE_LOCK_WAIT_ACK:
              log_printf( "Got ack in Lock wait ack\n" );
              nextState( STATE_LOCK_WAIT );
              break;
              
            case STATE_SET_PSM_WAIT_ACK:
              // unregister timeout
              system_timeout_unregister( &timeout );
              // fall-through

            case STATE_SET_PSM_WRITE:
              nextState( STATE_AFTER_PSM_WAIT );
              break;

            case STATE_SENDING_UBLOX_WAIT_ACK:
              log_printf( "Got ack in STATE_SENDING_UBLOX_WAIT_ACK, Going to Ready!\n" );
              nextState(STATE_SET_PSM_WRITE);
              break;
              
            default:
              nextState( STATE_READY );
              log_printf( "Ublox6 GPS: Ignoring a ACK-ACK packet because state is %d\n", state );
          }
          break;
          
        default:
          log_printf( "Ublox6 GPS: Ignoring a ACK packet with id %02x\n", id );
      }
      break;
      
    case UBLOX_CLASS_MON:
      switch( id )
      {
        case UBLOX_MON_VER:
          switch( state )
          {
            case STATE_POLLING_VERSION_WAITING:
              system_timeout_unregister( &timeout );  // only unregister timeout if waiting.
              // fall-through
              
            case STATE_POLLING_VERSION_SENDING:
              log_printf( "Got MON VER for version waiting/ending!\n");
              nextState( STATE_DISABLE_NMEA_WRITE );
              break;

            default:
              log_printf( "ublox6 gps: ignoring MON VER packet, state=%d\n", state );
          }
          break;
          
        default:
          log_printf( "Ublox6 GPS: Ignoring a MON packet with id %02x\n", id );
          break;
      }
      break;
      
    case UBLOX_CLASS_NAV:
      switch( id )
      {
        case UBLOX_NAV_POSLLH:
          if( positionCallback == NULL )
            log_printf( "ublox6 gps: got POSLLH, but no callback registered\n");
          else
          {
            position.longitude = ((uint32_t)readBuffer[8]) |
            ( ((uint32_t)readBuffer[9]) << 8) |
            ( ((uint32_t)readBuffer[10]) << 16) |
            ( ((uint32_t)readBuffer[11]) << 24);

            position.latitude = readBuffer[12] |
            ( ((uint32_t)readBuffer[13]) << 8) |
            ( ((uint32_t)readBuffer[14]) << 16) |
            ( ((uint32_t)readBuffer[15]) << 24);
            
            position.accuracy = readBuffer[24] |
            ( ((uint32_t)readBuffer[25]) << 8) |
            ( ((uint32_t)readBuffer[26]) << 16) |
            ( ((uint32_t)readBuffer[27]) << 24);
            
            positionCallback( &position );
          }
          break;
          
        case UBLOX_NAV_SOL:
          if( state == STATE_LOCK_WAIT )
          {
            GPSFixType fixType;
            int satCount;
            
            fixType = readBuffer[ 4 + 10 ];
            satCount = readBuffer[ 4 + 47 ];
            
            // actually, want > 4 here, changed to 3 for testing inside window
            if( (fixType > 0) && (satCount > 4) )
            {
              log_printf( "Got good fix, Fix: %d, Sat count=%d\n",
                     fixType, satCount );
              nextState( STATE_SET_PSM_WRITE );
            }
            else
              log_printf( "Waiting for good fix: Fix: %d, Sat count=%d\n",
                     fixType, satCount );
          }
          break;
          
        default:
          log_printf( "Ublox6 GPS: Ignoring a NAV packet with id %02x\n", id );
          break;
      }
      break;
      
    default:
      log_printf( "Received a UBlox packet of class %02x, with id %02x\n", class, id );
      break;
  }
}

static int ublox_calcChecksum( const uint8_t *data, int length )
{
  int ckA = 0, ckB = 0, i;
  
  for( i = 0; i < length; i++ )
  {
    ckA += data[ i ];
    ckB += ckA;
  }
  
  return (ckA & 0xff) | ((ckB & 0xff) << 8);
}

static void handleNMEA()
{
  // validate checksum
  // if valid, parse, handle
  // for now, ignore all NMEA messages
  
  // debug printout
  readBuffer[ readCursor-2 ] = '\0'; // remove CR LF
  
  log_printf( "N" ); // Received NMEA\n" ); // : %s\n", readBuffer );
}


#if 0
// we're done, handle received packet
switch( state )
{
  case POLLING_VERSION_WAITING:
    if( (readBuffer[0] == UBLOX_CLASS_MON) &&
       (readBuffer[1] == UBLOX_MON_VER) )
      state = STATE_DISABLE_SENTENCES;
      break;
    
    
}
#endif

// returns true if we should try writing again
static bool nextState( State newState )
{
  Error error;
  bool sendDone, rewrite = false;
  // State oldState;
  uint8_t disableNMEA[20] = { 1, /* UART port */
                              0, /* reserved d*/
                              0, 0, /* txReady */
                              0xd0, 0x08, 0x00, 0x00, /* mode 8Na */
                              9600 & 0xff, (9600 >> 8) & 0xff, (9600 >> 16) & 0xff, (9600 >> 24) & 0xff, /* baud rate */
                              0x03, 0x00, /* inProto mask (NMEA and UBlox) */
                              0x01, 0x00, /* outProto mask (UBlox) */
                              0x00, 0x00, /* reserved4 */
                              0x00, 0x00, /* reserved5 */ };
  uint8_t setPSMcyclic[2] = { 8, 1 };
  uint8_t navSolPollData[3] = { UBLOX_CLASS_NAV, UBLOX_NAV_SOL, 1 };
  uint32_t now;

  log_printf( "Ublox 6: nextState( %d '%s')\n", newState, stateNames[ newState ] );
  
  // oldState = state;
  state = newState;

  switch( newState )
  {
    case STATE_NMEA_SET_PROTOCOLS_WRITE:
      uBloxSendProcessedCount = 0;
      error = serial_writeFromBuffer( serial, (uint8_t *) nmeaSetProtocols,
                                      strlen( nmeaSetProtocols ), &uBloxSendProcessedCount, &sendDone );
      assert( error == NULL );
      
      rewrite = true;
      // Let the serviceWrite() function transition to STATE_POLLING_VERSION_SENDING when
      // done
      // newState =
      // newState = sendDone ? STATE_POLLING_VERSION_SENDING : STATE_POLLING_VERSION_SENDING;
      break;
      
    case STATE_POLLING_VERSION_SENDING:
      // message size will be packet size + 8
      prepareUBloxSendMessage( uBloxSendBuffer, sizeof( uBloxSendBuffer ),
                               UBLOX_CLASS_MON, UBLOX_MON_VER, 0, NULL);
      
      uBloxSendProcessedCount = 0;
      error = serial_writeFromBuffer( serial, uBloxSendBuffer, 8, &uBloxSendProcessedCount, &sendDone );
      assert( error == NULL );
      rewrite = true;
     
      // state = sendDone ? STATE_POLLING_VERSION_WAITING : STATE_POLLING_VERSION_SENDING;
      break;
      
    case STATE_POLLING_VERSION_WAITING:
      // set up timeout of 1 second
      now = system_timeMs_get();
      timeout.time = now + 1000;
      system_timeout_register( &timeout );
      printf( "Timeout time=%ld + 1000 = %ld\n", now, timeout.time );
      rewrite = false;
      break;
      
    case STATE_DISABLE_NMEA_WRITE:
      // 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B
      prepareUBloxSendMessage( uBloxSendBuffer, sizeof( uBloxSendBuffer ), UBLOX_CLASS_CFG, UBLOX_CFG_PRT, 20, disableNMEA );
      
      uBloxSendProcessedCount = 0;
      error = serial_writeFromBuffer( serial, uBloxSendBuffer, 20+8, &uBloxSendProcessedCount, &sendDone );
      assert( error == NULL );
      printf( "Disable NMEA Write 1: sendDone=%d, preocessedCount=%d\n", (int) sendDone, uBloxSendProcessedCount );

      rewrite = true;
      // state = sendDone ? STATE_DISABLE_NMEA_WAIT_ACK : // STATE_DISABLE_NMEA_WRITE;
      break;
      
    case STATE_DISABLE_NMEA_WAIT_ACK:
      // set up timeout of 1 second
      timeout.time = system_timeMs_get() + 1000;
      system_timeout_register( &timeout );

      rewrite = false;
      break; // do nothing
      
    case STATE_LOCK_WAIT_WRITE:
      // poll Navigation solution until fix and at least 5 satellites

      uBloxMessageLength = 8 + 3;
      uBloxSendProcessedCount = 0;
      prepareUBloxSendMessage( uBloxSendBuffer, sizeof( uBloxSendBuffer), UBLOX_CLASS_CFG,
                              UBLOX_CFG_MSG, 3, navSolPollData );
      // nextState( STATE_SENDING_UBLOX );
      rewrite = true;
      break;
      
    case STATE_LOCK_WAIT_ACK:
      rewrite = false;
      break;
      
    case STATE_LOCK_WAIT:
      rewrite = false;
      break;
      
    case STATE_SET_PSM_WRITE:
      prepareUBloxSendMessage( uBloxSendBuffer, sizeof( uBloxSendBuffer ), UBLOX_CLASS_CFG, UBLOX_CFG_RXM, 2, setPSMcyclic );
      
      uBloxSendProcessedCount = 0;
      error = serial_writeFromBuffer( serial, uBloxSendBuffer, 2+8, &uBloxSendProcessedCount, &sendDone );
      assert( error == NULL );
      
      rewrite = true;
      break;

    case STATE_SET_PSM_WAIT_ACK:
      // set up timeout of 1 second
      timeout.time = system_timeMs_get() + 1000;
      system_timeout_register( &timeout );
      rewrite = false;
      break;

    case STATE_AFTER_PSM_WAIT:
      timeout.time = system_timeMs_get() + 1000;
      system_timeout_register( &timeout );
      rewrite = false;
      break;
      
    case STATE_READY:
      // call user callback
      // state = newState; // or the callback won't see the stack as ready.

      if( !notifiedReady )
      {
        notifiedReady = true;
        readyCallback( NULL );
        rewrite = true; // just to be on the safe side.
      }
      else
        rewrite = false;
      break;
      // sendUBlox( &serial, UBLOX_CLASS_MON, UBLOX_MON_VER, 0, NULL );
      
    case STATE_SENDING_UBLOX:
      rewrite = true;
      break; // let serviceWrite select if we need wakeup because of PSM
      
    case STATE_SENDING_UBLOX_WAKEUP:
      // don't do anything, let serviceWrite handel it
      rewrite = true;
      break;
      
    case STATE_SENDING_UBLOX_PREWAIT:
      timeout.time = system_timeMs_get() + 500;
      system_timeout_register( &timeout );
      
      rewrite = false;
      break;
      
    case STATE_SENDING_UBLOX_SENDING:
      uBloxSendProcessedCount = 0;
      error = serial_writeFromBuffer( serial, uBloxSendBuffer, uBloxMessageLength, &uBloxSendProcessedCount, &sendDone );
      rewrite = true;
      break;
      
    case STATE_SENDING_UBLOX_WAIT_ACK:
      rewrite = false;
      break; // do nothing, should initiate a timeout here
      
    case STATE_OFF:
      // TODO: send Off command?
      rewrite = true;
      notifiedReady = false; // send ready again after off.
      break;
  }
  
  return rewrite;
}

static void prepareUBloxSendMessage( uint8_t *buffer, int bufferSize, UBloxClass class, UBloxID id,
                                      int packetLength, const uint8_t *ptr )
{
  int ckA, ckB, i;
  
  buffer[0] = 0xb5; // UBlox signature
  buffer[1] = 0x62; // UBlox signature
  buffer[2] = (uint8_t) class;
  buffer[3] = (uint8_t) id;
  buffer[4] = packetLength & 0xff;
  buffer[5] = packetLength >> 8;
  memcpy( &(buffer[6]), ptr, packetLength ); // TODO: Check that buffer won't overflow
  
  // calculate checksum
  ckA = 0; ckB = 0;
  
  // iteration 1: cKA = b0, ckB = b0
  // iteration 2: ckA = b0 + b1, ckB = b0 + b0 + b1
  for( i = 2; i < packetLength + 6; i++ )
  {
    ckA += buffer[i];
    ckB += ckA;
  }

  buffer[ packetLength + 6 ] = ckA & 0xff;
  buffer[ packetLength + 7 ] = ckB & 0xff;
}

Error gps_getPositions( PositionCallback callback )
{
  uint8_t data[3] = { UBLOX_CLASS_NAV, UBLOX_NAV_POSLLH, 1 };
  
  if( state != STATE_READY)
  {
    myError.cause = NULL;
    myError.description = "Not ready, please retry";
    myError.error = GPS_ERR_NOT_READY;
    myError.module = errorModule;
    myError.moduleName = "gps";
    
    return &myError;
  }
  
  positionCallback = callback;
  uBloxMessageLength = 8 + 3;
  prepareUBloxSendMessage( uBloxSendBuffer, sizeof( uBloxSendBuffer), UBLOX_CLASS_CFG,
                          UBLOX_CFG_MSG, 3, data );
  nextState( STATE_SENDING_UBLOX );
  
  return NULL;
}

Error gps_power_set( bool on )
{
  uint8_t data[8] = { 0, 0, 0, 0, 2, 0, 0, 0 };
  
  if( state != STATE_READY)
  {
    myError.cause = NULL;
    myError.description = "Not ready, please retry";
    myError.error = GPS_ERR_NOT_READY;
    myError.module = errorModule;
    myError.moduleName = "gps";
    
    return &myError;
  }
  
  uBloxMessageLength = 8 + 8;
  if( on )
  {
    // data[ 4 ] = 0;
    notifiedReady = false;
    nextState( STATE_POLLING_VERSION_SENDING );
  }
  else
  {
    data[4] = 2;
    
    prepareUBloxSendMessage( uBloxSendBuffer, sizeof( uBloxSendBuffer),
                            UBLOX_CLASS_RXM,
                            UBLOX_RXM_PMREQ, 8, data );
    nextState( STATE_SENDING_UBLOX );
  }
  return NULL;
}

