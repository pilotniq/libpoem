/*
  ubloxText.c
*/

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <poem/serial.h>
#include <poem/unix/serial_unix.h>

typedef enum 
  {
    UBLOX_CFG_PRT = 0x00,
    UBLOX_MON_VER = 0x04
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


char buf[8192];

static void sendUBlox( SerialChannel serial, UBloxClass class, UBloxID id, uint16_t packetLength, uint8_t *data );
static Error sendNMEA( SerialChannel serial, const char *msg );
static void sendNMEAenableUBlox( SerialChannel serial );
static Error blockingRead( SerialChannel serial, char *buffer, int length );

int main( int argc, char **argv )
{
  struct sSerialChannel serial;
  Error error;
  int i;

  error = unix_serial_init( &serial, argv[1], 9600 );
  if( error != NULL )
  {
    fprintf( stderr, "serial_init failed: %s (%d) %s\n", error->moduleName, error->error, error->description );
    return -1;
  }

  printf( "fd=%d\n", serial.fd );

  int length;
  int packetLength;
  char ch;

  
  // 
  // sendNMEAenableUBlox( &serial );

  for(;;) {
    // read one character
    length = 1;
    error = serial_read( &serial, &ch, &length );
    if( error != NULL )
    {
      if( error->error == EAGAIN )
      {
        printf( "." );
        continue;
      }
      else
      {
        fprintf( stderr, "serial_read one byte error\n" );
        goto FAIL;
      }
    }

    assert( error == NULL );

    if( length < 1 )
      continue;

    if( ch == '$' )
    {
      int bufIndex;
      // assume NMEA message
      // read until <LF>, discard
      
      // sleep( 1 ); // to let the rest of the bytes arrive

      for( bufIndex = 0; ch != '\n'; bufIndex++ )
      {
        do
        {
          length = 1;
          error = serial_read( &serial, &ch, &length );
          if( error != NULL )
          {
            printf( "Error in serial_read of NMEA\n" );
            goto FAIL;
          }
        } while( length < 1 );
        buf[bufIndex] = ch;
      }
      buf[ bufIndex ] = '\0';
      printf( "NMEA message: '%s'\n", buf );
      
      // turn off NMEA sentences
      error = sendNMEA( &serial, "PUBX,40,GLL,0,0,0,0,0,0" );
      printf( "After sendNMEA\n" );
      if( error != NULL )
        goto FAIL;
      // sendNMEA( &serial, "PUBX,40,GGA,0,0,0,0,0,0" );
      // sendNMEA( &serial, "PUBX,40,GSA,0,0,0,0,0,0" );
      // sendNMEA( &serial, "PUBX,40,RMC,0,0,0,0,0,0" );
      // sendNMEA( &serial, "PUBX,40,GSV,0,0,0,0,0,0" );
      // sendNMEA( &serial, "PUBX,40,VTG,0,0,0,0,0,0" );
      
      // Poll for version
      sendUBlox( &serial, UBLOX_CLASS_MON, UBLOX_MON_VER, 0, NULL );
      
      //      sendNMEAenableUBlox( &serial );
    }
    else if( ((unsigned char) ch) == 0xb5 )
    {
      printf( "Got ublox !?\n" );
      
      // assume start of uBlox packet
      length = 5;
      error = serial_read( &serial, buf, &length );
      assert( error == NULL );
      
      if( buf[0] == 0x62 )
      {
        int packetLength = buf[3] + (buf[4] << 8);
        char payload[1024];
        char checksum[2];
        
        length = length + 2;
        error = serial_read( &serial, &(buf[5]), &length );
        assert( error == NULL );
        
        printf( "UBlox: class=%d, id=%d, length=%d\n", buf[1], buf[2], packetLength );
        // read rest of package
        length = packetLength;
        
        blockingRead( &serial, payload, length );
        
        printf( "  Payload: " );
        for( i = 0; i < packetLength; i++ )
          printf( "%02x ", payload[i] );
        
        printf( "\n" );
        
        // read checksum (2 bytes )
        blockingRead( &serial, checksum, 2 );
      }
    }
    else
      printf( "got len=%d, '%c' (%d)\n", (int) length, ch, ch );
    
    // len = read( fd, &buf[0], 8192 );
    // if( len > 0 ) write(1,buf,len);
    // sendUBlox( &serial, UBLOX_CLASS_CFG, UBLOX_CFG_PRT, 0, NULL );
    
    // printf( "UBlox sent\n" );
  }
  
  return 0;
  
FAIL:
  fprintf( stderr, "failed: %s (%d) %s\n", error->moduleName, error->error, error->description );
  return -1;
}

static Error blockingRead( SerialChannel serial, char *buffer, int length )
{
  int i;
  int remainingLength;
  Error error;
  
  for( i = 0, remainingLength = length; remainingLength > 0; )
  {
    length = remainingLength;
    error = serial_read( serial, &(buffer[ i ]), &length );
    assert( error == NULL );

    i+= length;
    remainingLength -= length;
  }

  return NULL;
}

static void sendUBlox( SerialChannel serial, UBloxClass class, UBloxID id, uint16_t packetLength, uint8_t *data )
{
  uint8_t preamble[6];
  uint8_t ckA, ckB;
  int i;
  int length;
  Error error;

  preamble[0] = 0xb5;
  preamble[1] = 0x62;
  preamble[2] = (uint8_t) class;
  preamble[3] = id;
  preamble[4] = packetLength & 0xff;
  preamble[5] = packetLength >> 8;
  
  length = 6;
  error = serial_write( serial, (char *) preamble, &length );
  assert( error == NULL );
  assert( length == 6 );

  length = packetLength;
  error = serial_write( serial, (char *) data, &length );
  assert( error == NULL );
  assert( length == packetLength );

  printf( "UBlox: " );
  for( i = 0; i < 6; i++ )
  {
    printf( "%02x ", preamble[i] );
  }

  for( i = 0; i < length; i++ )
  {
    printf( "%02x ", data[i] );
  }

  // calculate checksum
  ckA = 0; ckB = 0;

  // iteration 1: cKA = b0, ckB = b0
  // iteration 2: ckA = b0 + b1, ckB = b0 + b0 + b1
  for( i = 2; i < 6; i++ )
  {
    ckA += preamble[i];
    ckB += ckA;

    //    printf( "ckB = %d\n", ckB );
  }

  for( i = 0; i < length; i++ )
  {
    ckA += data[i];
    ckB += ckA;

    //    printf( "ckB = %d\n", ckB );
  }

  length = 1;
  error = serial_write( serial, (char *) &ckA, &length );
  assert( error == NULL );
  assert( length == 1 );

  length = 1;
  error = serial_write( serial, (char *) &ckB, &length );
  assert( error == NULL );
  assert( length == 1 );
}

static void sendNMEAenableUBlox( SerialChannel serial )
{
  sendNMEA( serial, "PUBX,41,1,0003,0003,9600,0" ); // 25\r\n";
#if 0
  // send NMEA command to enable UBlox
  char *msg = "$PUBX,41,1,0003,0003,9600,0*"; // 25\r\n";
  uint8_t checksum = 0;
  char ch;
  int i;
  int length;
  Error error;

  printf( "Checksumming: " );
  for( i = 1; i < strlen( msg ) - 1; i++ )
  {
    checksum = checksum ^ msg[ i ];
    printf( "%c", msg[i ]);
  }
  
  length = strlen( msg );
  error = serial_write( serial, msg, &length );
  assert( error == NULL );
  assert( length == strlen( msg ) );
	  
  sprintf( (char *) buf, "%02x\r\n", checksum );

  length = 4;
  error = serial_write( serial, buf, &length );
  assert( error == NULL );
  assert( length == 4 );
  
  printf( "Sent: %s%s", msg, buf );

  sendUBlox( serial, UBLOX_CLASS_CFG, UBLOX_CFG_PRT, 0, NULL );

  printf( "UBlox sent\n" );
#endif
}

// Pass message without first '$' and checksum
static Error sendNMEA( SerialChannel serial, const char *msg )
{
  uint8_t checksum = 0;
  char buf[6];
  int i, length;
  const char dollar = '$';
  Error error;

  for( i = 0; i < strlen( msg ); i++ )
    checksum = checksum ^ msg[ i ];

  length = 1;
  error = serial_write( serial, &dollar, &length );
  if( error != NULL )
    return error;
  assert( length == 1 );

  length = strlen( msg );
  error = serial_write( serial, msg, &length );
  assert( error == NULL );
  assert( length == strlen( msg ) );
	  
  sprintf( (char *) buf, "*%02X\r\n", checksum );
  // buf[1] = toupper( buf[1] );
  assert( strlen( buf ) == 5 );

  length = 5;
  error = serial_write( serial, buf, &length );
  assert( error == NULL );
  assert( length == 5 );

  return NULL;
}
