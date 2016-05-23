//
//  serial.c
//  
//
//  Created by Erland Lewin on 2016-05-17.
//
//

#include <poem/serial.h>

Error serial_readToBuffer( SerialChannel channel, uint8_t *buffer, int readLength, int *cursor,
                           bool *done)
{
  Error error;
  int readCount = readLength - *cursor;
  
  error = serial_read( channel, &(buffer[ *cursor ]), &readCount );
  if( error != NULL )
    return error;

  *cursor += readCount;

  *done = (*cursor >= readLength);

  return NULL;
}

Error serial_writeFromBuffer( SerialChannel channel, const uint8_t *buffer, int bufferSize, int *cursor,
                              bool *done)
{
  Error error;
  int writeCount = bufferSize - *cursor;
  
  error = serial_write( channel, &(buffer[ *cursor ]), &writeCount );
  if( error != NULL )
    return error;
  
  *cursor += writeCount;
  *done = (*cursor >= bufferSize );
  
  return NULL;
}
