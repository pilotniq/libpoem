/*
 * PiChart Protoboard for GPS dongle
 */

// Defines the number of LEDs. In this example, there is a single RGB LED.
#define LEDS_NUMBER    0
// Defines which PIOs control the color of the LED.
#define LED_START      0
#define LED_STOP       0
#define LEDS_MASK      (0)
// Defines which LEDs are lit when the signal is low. In this example,
// all LEDs are lit.
#define LEDS_INV_MASK  LEDS_MASK
// Defines the user buttons. In this example, there are no user buttons.
#define BUTTONS_NUMBER 0
#define BUTTONS_MASK   0x00000000

// Defines the UART connection with J-Link.
#define RX_PIN_NUMBER  5
#define TX_PIN_NUMBER  6
#define CTS_PIN_NUMBER -1
#define RTS_PIN_NUMBER -1
#define HWFC           false

#define NRF_LOG_USES_UART 0
// I have no idea what the PPM is of the low frequency crystal
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_100_PPM}

