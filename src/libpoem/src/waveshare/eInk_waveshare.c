#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <poem/eink_waveshare.h>
#include <poem/gpio.h>
#include <poem/spi.h>
#include <poem/system.h>

typedef enum  { EINK_COMMAND_DRIVER_OUTPUT_CONTROL = 0x01, 
    // from https://gitlab.com/hooeezit/waveshare-epaper-nrf52-dk-demo/-/blob/master/waveshare_epd.c
                EINK_COMMAND_BOOSTER_SOFT_START_CONTROL = 0x0C,
                EINK_COMMAND_DEEP_SLEEP = 0x10,
                                  EINK_COMMAND_DATA_ENTRY_MODE_SETTING = 0x11,
                                  EINK_COMMAND_SW_RESET = 0x12, 
                                  EINK_COMMAND_0x18 = 0x18,
                                  EINK_COMMAND_MASTER_ACTIVATION = 0x20,
                                  EINK_COMMAND_DISPLAY_UPDATE_CONTROL_2 = 0x22,
                                  EINK_COMMAND_WRITE_RAM_BW = 0x24,
                EINK_COMMAND_WRITE_VCOM_REGISTER = 0x2c,
                EINK_COMMAND_SET_DUMMY_LINE_PERIOD = 0x3a,
                EINK_COMMAND_SET_GATE_TIME = 0x3b,
                                  EINK_COMMAND_BORDER_WAVE_FROM = 0x3c,
                                  EINK_COMMAND_SET_RAM_X_START_END = 0x44,
                                  EINK_COMMAND_SET_RAM_Y_START_END = 0x45,
                                  EINK_COMMAND_SET_RAM_X_ADDRESS_COUNTER = 0x4e,
                                  EINK_COMMAND_SET_RAM_Y_ADDRESS_COUNTER = 0x4f,
                                  EINK_COMMAND_TERMINATE_FRAME_READ_WRITE = 0xff } WAVESHARE_COMMANDS;

/*
 * static function prototypes 
 */


static void waitUntilIdle( EInkDisplay display );
static void sendCommand( EInkDisplay display, uint8_t command);
static void sendData( EInkDisplay display, uint8_t data);
static Error spiTransfer( EInkDisplay display, uint8_t byte );
static void refreshDisplay( EInkDisplay display );
static void waitUntilIdle( EInkDisplay display );

static void masterActivation( EInkDisplay display );
static void displayUpdateControl2( EInkDisplay display, uint8_t param );
static void terminateFrameReadWrite( EInkDisplay display );

/*
 * Start of code
 * SPI must be MSB first, mode 0
 */
void eInk_waveshare_init( sEInkDisplay *display, 
                           SPIchannel spiChannel, 
                           int resetPin, int dcPin, int csPin, int busyPin, int width, int height )
{
    display->spi = spiChannel;
    display->width = width;
    display->height = height;
    display->busyPin = busyPin;
    display->dcPin = dcPin;
    display->resetPin = resetPin;
    display->csPin = csPin;

    // todo: check for errors
    gpio_configure( csPin, GPIO_DIRECTION_OUTPUT, GPIO_PULL_NONE );
    gpio_configure( resetPin, GPIO_DIRECTION_OUTPUT, GPIO_PULL_NONE );
    gpio_configure( dcPin, GPIO_DIRECTION_OUTPUT, GPIO_PULL_NONE );
    gpio_configure( busyPin, GPIO_DIRECTION_INPUT, GPIO_PULL_NONE ); 
    
    // SPI.begin();
    // SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

    eInk_waveshare_reset( display ); // this is a hardware reset
    waitUntilIdle( display );

    sendCommand( display, EINK_COMMAND_SW_RESET );
    waitUntilIdle( display );

    sendCommand( display, EINK_COMMAND_DRIVER_OUTPUT_CONTROL); //Driver output control
    sendData( display, 0xC7 ); 
    sendData( display, 0x00 );
    /* Unclear what this byte should be. 
       https://gitlab.com/hooeezit/waveshare-epaper-nrf52-dk-demo/-/blob/master/waveshare_epd.c sends 0
       So does https://github.com/soonuse/epd-library-arduino/blob/master/1.54inch_e-paper/epd1in54/epd1in54.cpp
        Let's also use zero
        */
    sendData( display, 0x01 );

#if 0
    // soft start? from 
    sendCommand( display, EINK_COMMAND_BOOSTER_SOFT_START_CONTROL);
    sendData( display, 0xd7 );
    sendData( display, 0xd6 );
    sendData( display, 0x9d );

    sendCommand( display, EINK_COMMAND_WRITE_VCOM_REGISTER);
    sendData( display, 0xa8 );

    sendCommand( display, EINK_COMMAND_SET_DUMMY_LINE_PERIOD);
    sendData( display, 0x1A);
 
    sendCommand( display, EINK_COMMAND_SET_GATE_TIME );
    sendData( display, 0x08);
#endif

    sendCommand( display, EINK_COMMAND_DATA_ENTRY_MODE_SETTING); //data entry mode
    // 1 = decrement Y, increment X
    // 3 = increment Y, increment X
    sendData( display, 0x01); // can be modified to change orientation? Change from 1 to 3 2020-03-10

    sendCommand( display, EINK_COMMAND_SET_RAM_X_START_END);
    sendData( display, 0x00);
    sendData( display, 0x18);    //0x0C-->(18+1)*8=200 (todo: (take width / 8- 1) 

    sendCommand( display, EINK_COMMAND_SET_RAM_Y_START_END); //set Ram-Y address start/end position
    sendData( display, 0xC7);   //0xC7-->(199+1)=200. todo: width - 1
    sendData( display, 0x00 );
    sendData( display, 0x00 );
    sendData( display, 0x00 );

    // this is not used in waveshare example!?
    sendCommand( display, EINK_COMMAND_BORDER_WAVE_FROM ); //BorderWavefrom (undocumented in data sheet)
    sendData( display, 0x01 );

    sendCommand( display, EINK_COMMAND_0x18 ); // undocumented command 
    sendData( display, 0x80 );

    // displayUpdateControl2( display, 0XB1 ); // 0xB1 = Load temperature value, loat LUT with display mode 1, disablel clock

    // c0 = Enable clock signal →Enable Analog
    displayUpdateControl2( display, 0xB1 ); // 0xB1 = Load temperature value, loat LUT with display mode 1, disablel clock. Have seen 0xc1 used oto

    masterActivation( display );
 
    sendCommand( display, EINK_COMMAND_SET_RAM_X_ADDRESS_COUNTER);   // set RAM x address count to 0;
    sendData( display, 0x00 );

    sendCommand( display, EINK_COMMAND_SET_RAM_Y_ADDRESS_COUNTER);   // set RAM y address count to 0X199;
    sendData( display, 0xC7);
    sendData( display, 0x00);

    waitUntilIdle( display );

    /* EPD hardware init end */      
}

void eInk_waveshare_reset( EInkDisplay display )
{
    gpio_write( display->resetPin, true);
    system_delayMs(200);

    gpio_write( display->resetPin, false);                //module reset
    system_delayMs(10);
    
    gpio_write( display->resetPin, true);
    system_delayMs(200);
}

void eInk_waveshare_clear( EInkDisplay display )
{
    int w, h;

    // below could be simplified to (display->width + 7) % 8 ?
    w = (display->width % 8 == 0)? (display->width / 8 ): (display->width / 8 + 1);

    h = display->height;
    sendCommand( display, EINK_COMMAND_WRITE_RAM_BW );
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            sendData( display, 0x35); // 0011 0101 = 0x35
        }
    }

    //DISPLAY REFRESH
    refreshDisplay( display );
}

void eInk_waveshare_display( EInkDisplay display, const uint8_t *frameBuffer )
{
    int w = (display->width % 8 == 0)? (display->width / 8 ): (display->width / 8 + 1);
    int h = display->height;

    if (frameBuffer != NULL) {
        sendCommand( display, EINK_COMMAND_WRITE_RAM_BW );

        for (int j = 0; j < h; j++) {
            for (int i = 0; i < w; i++) {
                sendData( display, frameBuffer[i + j * w] );
            }
        }
    }

    masterActivation( display );

    waitUntilIdle( display );  // todo: add Timeout for robustness

    //DISPLAY REFRESH
    // refreshDisplay( display );

    // this is from https://gitlab.com/hooeezit/waveshare-epaper-nrf52-dk-demo/-/blob/master/waveshare_epd.c
    // terminateFrameReadWrite( display );
}


/*
Error eInk_waveshare_displayPartBaseImage( EInkDisplay display, const uint8_t *frameBuffer );
Error eInk_waveshare_displayPart( EInkDisplay display, const uint8_t *frameBuffer );
*/
void eInk_waveshare_sleep( EInkDisplay display )
{
    sendCommand( display, EINK_COMMAND_DEEP_SLEEP ); //enter deep sleep
    sendData( display, 0x01 );

    system_delayMs( 200 );

    gpio_write( display->resetPin, false);
}

/*
 *  Utility functions
 */
static void refreshDisplay( EInkDisplay display )
{
    //DISPLAY REFRESH
    // 0xf7 = Enable analog, load temperature value, display mode 1, disable analog, disable osc
    // 0xc4 = ?
    displayUpdateControl2( display, 0xF7); // nrf example uses C4, was F7  
 
    masterActivation( display );

    waitUntilIdle( display );  // todo: add Timeout for robustness
}

// Todo: make non-blocking
static void waitUntilIdle( EInkDisplay display )
{
    while( gpio_read( display->busyPin ) )      //LOW: idle, HIGH: busy
        system_delayUs( 1 );

    system_delayUs( 1 );
}

/**
 *  @brief: basic function for sending commands
 */
static void sendCommand( EInkDisplay display, uint8_t command)
{
    Error error;

    gpio_write( display->dcPin, false );
 
    error = spiTransfer( display, command );
    assert( error == NULL );
}

/**
 *  @brief: basic function for sending data
 */
static void sendData( EInkDisplay display, uint8_t data)
{
    gpio_write( display->dcPin, true);
    spiTransfer( display, data );
 }

static Error spiTransfer( EInkDisplay display, uint8_t byte )
{
    Error error;

    gpio_write( display->csPin, false);
    error = spi_transfer( display->spi, &byte, 1, NULL, 0 );
    gpio_write( display->csPin, true);

    return error;
}

/*
 * Low level commands 
 */
static void masterActivation( EInkDisplay display )
{
  sendCommand(display, EINK_COMMAND_MASTER_ACTIVATION );

  // waitUntilIdle( display );  // todo: add Timeout for robustness
}

static void displayUpdateControl2( EInkDisplay display, uint8_t param )
{
   sendCommand(display, EINK_COMMAND_DISPLAY_UPDATE_CONTROL_2);
   sendData(display, param );
}

static void terminateFrameReadWrite( EInkDisplay display )
{
    sendCommand(display, EINK_COMMAND_TERMINATE_FRAME_READ_WRITE);
}