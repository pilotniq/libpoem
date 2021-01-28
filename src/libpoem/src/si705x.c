/*
   Silicon Labs Si705x temperature sensor chips
   */
#include <stdint.h>
#include <stddef.h> // for NULL

#include <poem/i2c_master.h>
#include <poem/si705x.h>

#define I2C_ADDRESS 0x40

static sError error;
int module_error_id;

void si705x_init()
{
    module_error_id = err_registerModule();
    error.module = module_error_id;
}

// serial must be an array of 8 bytes
Error si705x_readSerial( I2Cchannel channel, uint8_t *serial )
{
    Error error;
    const uint8_t write1[] = { 0xfa, 0x0f };
    const uint8_t write2[] = { 0xfc, 0xc9 };
    uint8_t sna[8];
    uint8_t snb[6];

    error = i2c_write( channel, I2C_ADDRESS, sizeof( write1 ), write1, false );
    if( error != NULL )
        return error;

    error = i2c_read( channel, I2C_ADDRESS, sizeof( sna ), sna );
    if( error != NULL )
        return error;

    error = i2c_write( channel, I2C_ADDRESS, sizeof( write2 ), write2, false );
    if( error != NULL )
        return error;

    error = i2c_read( channel, I2C_ADDRESS, sizeof( snb ), snb );
    if( error != NULL )
        return error;

    // TODO: check checksum

    serial[0] = sna[0];
    serial[1] = sna[2];
    serial[2] = sna[4];
    serial[3] = sna[6];
    serial[4] = snb[0];
    serial[5] = snb[1];
    serial[6] = snb[3];
    serial[7] = snb[4];

    return NULL;
}

Error si705x_setResolution( I2Cchannel channel, Si705x_Resolution resolution )
{
    uint8_t writeBuf[2];
    Error my_error;

    writeBuf[0] = 0xe6;
    if( resolution & 0x2)
        writeBuf[1] = 0x80;
    else
        writeBuf[1] = 0;

    if( resolution & 0x1 )
        writeBuf[1] |= 1;

    my_error = i2c_write( channel, I2C_ADDRESS, sizeof( writeBuf ), writeBuf, false );
    if( my_error != NULL )
        return my_error;

    return NULL;
}
    
// TODO: add timeout support
Error si705x_readNoHold( I2Cchannel channel, Si705x_Temperature *temperature )
{
    uint8_t command = 0xf3;
    uint8_t readBuf[3];
    uint16_t tempCode;

    Error my_error;

    my_error = i2c_write( channel, I2C_ADDRESS, 1, &command, false );
    if( my_error != NULL )
        return my_error;

    do
    {
        my_error = i2c_read( channel, I2C_ADDRESS, 3, readBuf );
        if( my_error != NULL && 
            !(error.module == i2c_getErrorModule( channel ) && 
              error.error == I2C_ERR_NACK_AFTER_ADDRESS) )
            break;
    } while( my_error != NULL );

    if( my_error != NULL )
        return &error;

    // convert temperature
    tempCode = (readBuf[0] << 8) | readBuf[1];

    // temperature range is -40 to +125
    // return temperature * 100
    *temperature = ((((int32_t) 17572) * tempCode) >> 16) - 4685;

    return NULL;
}
