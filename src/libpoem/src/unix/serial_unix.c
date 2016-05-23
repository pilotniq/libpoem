/*
 * serial_osx.c
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h> // for debugging
#include <termios.h>
#include <sys/ioctl.h>

#include "poem/error.h"
#include "poem/serial.h"

#include "poem/unix/error_unix.h"
#include "poem/unix/serial_unix.h"
#include "poem/unix/system_unix.h"

static sError error;

Error unix_serial_init( SerialChannel channel, const char *device, speed_t baudRate )
{
  struct termios tio;
  int err;

  // get module error number
  unix_init();

  channel->fd = open( device, O_NOCTTY | O_NONBLOCK | O_RDWR );

  if( channel->fd < 0 )
  {
    unix_makeError( &error );

    return &error;
  }

  tcgetattr( channel->fd, &tio );

  cfmakeraw(&tio);

  assert( ioctl( channel->fd, TIOCEXCL ) != -1 );

  err = cfsetispeed( &tio, (speed_t) baudRate );
  assert( err == 0 );
  err = cfsetospeed( &tio, (speed_t) baudRate );
  assert( err == 0 );

  tio.c_oflag &= ~OPOST;

  //
  // Input flags - Turn off input processing
  //
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  //
  tio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
		   INLCR | PARMRK | INPCK | ISTRIP | IXON);


  tio.c_cflag &= ~CSIZE;
  tio.c_cflag &= ~CCTS_OFLOW;
  tio.c_cflag &= ~CRTS_IFLOW;

  tio.c_cflag |= CS8; //        | // Use 8 bit words
  tio.c_cflag |= CLOCAL;
		  //		  CCTS_OFLOW |        // CTS flow control of output
		  //		  CRTS_IFLOW);        // RTS flow control of input
  
  tio.c_cflag &= ~PARENB; // disable parity
  tio.c_cflag &= ~CSTOPB; // one stop bit

  //
  // One input byte is enough to return from read()
  // Inter-character timer off
  //
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 0;

  err = tcsetattr( channel->fd, TCSANOW, &tio );
  assert( err == 0 );

  system_addFD( channel->fd );
  
  return NULL;
}

/*
 * Start asynchronous write. Copies data from 'buffer' to internal buffer. 
 *
 * Parameter: length: input: amount of data in buffer. Output: actual bytes written, can be less if internal
 *   buffer is filled.
 *
 */
Error serial_write( SerialChannel channel, const uint8_t *buffer, int *length )
{
  ssize_t writeLen;
  int bytes;

  // printf( "serial_write: fd=%d, length=%d\n", channel->fd, *length );

  // test available to write
  ioctl( channel->fd, TIOCOUTQ, &bytes );

  // printf( "serial_write: %d bytees waiting to write\n", bytes );

  // TODO: Change EAGAIN to return and ask caller to try again
  do
  {
    writeLen = write( channel->fd, &(buffer[0]), *length );
    // if( (writeLen < 0) && (errno == EAGAIN) )
    //  sleep( 1 );
  } while( (writeLen < 0) && (errno == EAGAIN) );

  // printf( "serial_write: wrote. writeLen=%d\n", (int) writeLen );
  if( writeLen < 0 )
  {
    // printf( "calling unix_makeerror, errno=%d\n", errno );
    unix_makeError( &error );

    return &error;
  }

  *length = writeLen;

  return NULL;
}

/*
 * Non-blocking read
 * 
 * Parameters: length: input: size of buffer. Output: number of bytes placed in buffer 
 * 
 */
Error serial_read( SerialChannel channel, uint8_t *buffer, int *length )
{
  ssize_t readLen;

  readLen = read( channel->fd, buffer, *length );

  if( readLen < 0 )
  {
    if( errno == EAGAIN )
      readLen = 0;
    else
    {
      unix_makeError( &error );

      return &error;
    }
  }
  
  *length = readLen;

  return NULL;
}
