/*
 * pins_efm8bb.c
 *
 * Handle crossbar and pin assignment
 *
 *  Created on: 11 aug. 2017
 *      Author: erlandlewin
 */

#include <assert.h>
#include <SI_EFM8BB1_Register_Enums.h>

#include <poem/error.h>
#include <poem/efm8bb/pins_efm8bb.h>

/*
 * Static function prototypes
 */
static void setAnalogInMode( int pin );

static void setDigitalMode( int pin );
static void setPushPull( int pin );

static void disablePullUp( int pin );

static void skipPins( int *maxPin, int pin, uint8_t *newP0Skip, uint8_t *newP1Skip );

static int findPinByFunction( int pinCount, PinFunction *functions, PinFunction function );

// we have 15 pins in the QFN-20 package

void gpio_set( int pin, bool value )
{
	if( pin < 8 )
	{
		if( value )
			P0 |= (1 << pin);
		else
		{
			uint8_t mask;

			mask = ~(1 << pin);
			P0 &= mask;
		}
	}
	else
	{
		if( value )
			P1 |= (1 << (pin - 8));
		else
			P1 &= ~(1 << (pin - 8));
	}
}
// TODO: Fix error handling
Error pins_efm8bb_set_configure( int pinCount, const PinFunction pinFunctions[] )
{
	int pin, maxPin = -1, i;
	uint8_t newP0Skip = 0, newP1Skip = 0, newXBR0 = 0;

	// do crossbar

	// figure out which pins to skip
	assert( pinCount < 16 );

	// Don't yet support SPI.
	// Find the SDA pin
	pin = findPinByFunction( pinCount, pinFunctions, PINFUNCTION_I2C_SDA );

	if( pin >= 0 )
	{
		skipPins( &maxPin, pin, &newP0Skip, &newP1Skip );

		// do we need to set these pins as digital?
		newXBR0 |= XBR0_SMB0E__ENABLED;

		pin = findPinByFunction( pinCount, pinFunctions, PINFUNCTION_I2C_SCL );
		assert( pin > maxPin );

		skipPins( &maxPin, pin, &newP0Skip, &newP1Skip );
	}

	// skip the rest of the pins
	skipPins( &maxPin, 16, &newP0Skip, &newP1Skip );

	// Set P0MDIN, P1MDIN
	for( i = 0; i < pinCount; i++ )
	{
		switch( pinFunctions[i] )
		{
		  case PINFUNCTION_ADC:
			  setAnalogInMode( i );
			  break;

		  case PINFUNCTION_I2C_SDA:
		  case PINFUNCTION_I2C_SCL:
			  setDigitalMode( i );
			  break;

		  case PINFUNCTION_GPIO_INPUT:
			  setDigitalMode( i );
			  disablePullUp( i );
			  break;

		  case PINFUNCTION_GPIO_OUTPUT:
			  setDigitalMode( i );
			  setPushPull( i );
			  break;
		}
	}

	// Ignore PnMASK; initialized to ignore
	// Ignore PnMAT; all pins are ignored

	P0SKIP = newP0Skip;
	P1SKIP = newP1Skip;
	XBR0 = newXBR0;
	// Enable crossbar and weak pull-ups
	XBR2 = XBR2_WEAKPUD__PULL_UPS_ENABLED | XBR2_XBARE__ENABLED;

	// All ports low drive strength
	PRTDRV = 0;

	return NULL;
}

static void setAnalogInMode( int pin )
{
  if( pin < 8 )
	  P0MDIN = P0MDIN & ~(1 << pin);
  else if( pin < 16 )
	  P1MDIN = P1MDIN & ~(1 << (pin - 8));
}

static void setDigitalMode( int pin )
{
  if( pin < 8 )
	  P0MDIN = P0MDIN | (1 << pin);
  else if( pin < 16 )
	  P1MDIN = P1MDIN | (1 << (pin - 8));
}

static void setPushPull( int pin )
{
	  if( pin < 8 )
		  P0MDOUT = P0MDOUT | (1 << pin);
	  else if( pin < 16 )
		  P1MDOUT = P1MDOUT | (1 << (pin-8));
}

static void disablePullUp( int pin )
{
	  if( pin < 8 )
		  P0 = P0 | (1 << pin);
	  else if( pin < 16 )
		  P0 = P0 | (1 << (pin-8));
}

static void skipPins( int *maxPin, int pin, uint8_t *newP0Skip, uint8_t *newP1Skip )
{
	int i;

	// skip all pins before sda
	for( i = *maxPin + 1; i < pin; i++ )
		if( i < 8 )
			*newP0Skip = *newP0Skip | (1 << i);
		else
			*newP1Skip = *newP1Skip | (1 << (i - 8));

	*maxPin = pin;
}

static int findPinByFunction( int pinCount, PinFunction *functions, PinFunction function )
{
	int i;

	for( i = 0; i < pinCount; i++ )
		if( functions[ i ] == function )
			return i;

	return -1;
}
