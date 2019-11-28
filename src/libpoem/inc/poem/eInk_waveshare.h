/*
 * Waveshare E-Ink display driver
 */

#include <stdint.h>

#include <poem/error.h>
#include <poem/spi.h>

typedef struct sEInkDisplay
{
  SPIchannel spi;
  int height;
  int width;
  int busyPin;
  int dcPin;
  int resetPin;
  int csPin;
} sEInkDisplay, *EInkDisplay;

void eInk_waveshare_init( sEInkDisplay *display, SPIchannel spiChannel, 
                           int resetPin, int dcPin, int csPin, int busyPin, int width, int height );
void eInk_waveshare_reset( EInkDisplay display );
void eInk_waveshare_clear( EInkDisplay display );
void eInk_waveshare_display( EInkDisplay display, const uint8_t *frameBuffer );
/*
Error eInk_waveshare_displayPartBaseImage( EInkDisplay display, const uint8_t *frameBuffer );
Error eInk_waveshare_displayPart( EInkDisplay display, const uint8_t *frameBuffer );
*/
void eInk_waveshare_sleep( EInkDisplay display );
