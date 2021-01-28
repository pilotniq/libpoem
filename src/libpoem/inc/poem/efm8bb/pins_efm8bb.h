#ifndef PINS_EFM8BB_H
#define PINS_EFM8BB_H

#include <stdbool.h>
#include <poem/error.h>

typedef enum { PINFUNCTION_UNUSED, PINFUNCTION_I2C_SDA, PINFUNCTION_I2C_SCL,
	       PINFUNCTION_ADC, PINFUNCTION_GPIO_INPUT, PINFUNCTION_GPIO_OUTPUT,
} PinFunction;

Error pins_efm8bb_set_configure( int pinCount, PinFunction *pinFunctions );
void gpio_set( int pin, bool value );

#endif
