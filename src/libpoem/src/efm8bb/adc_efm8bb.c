/*
 * adc_efm8bb.c
 *
 *  Created on: 15 aug. 2017
 *      Author: erlandlewin
 */

#include <assert.h>

#include <SI_EFM8BB1_Register_Enums.h>

#include <poem/error.h>
#include <poem/efm8bb/adc_efm8bb.h>

static ADCchannel currentChannel = NULL;

/*
 * Start of code
 */
Error efm8bb_adc_create_burst( ADCchannel adcChannel, int pin, int resolution_bits
		/*, uint16_t ksps */ )
{
	assert( pin < 32 );

	adcChannel->pin = pin;
	adcChannel->state = ADCSTATE_IDLE;
	adcChannel->lastValue = 0xffff;

	ADC0MX = pin & 0x1f;

	 // ADSC (SAR Clock Divider) = 0x00
	 // HFOSC is used in burst mode, it runs at 24.5 MHz.
	 // In low power mode, SARCLK should be max 4 MHz.
	 // We therefore want to divide the HFOSC by 9 = 8 + 1, therefore ADSC should be 8
	 // AD8BE (8-Bit Mode Enable) = NORMAL (ADC0 operates in 10-bit or 12-bit
	 //     mode (normal operation).)
	 // ADGN (Gain Control) = GAIN_0P5 (The on-chip PGA gain is 0.5.)
	 // ADTM (Track Mode) = TRACK_NORMAL (Normal Track Mode. When ADC0 is
	 //     enabled, conversion begins immediately following the start-of-
	 //     conversion signal.)
	ADC0CF = (8 << ADC0CF_ADSC__SHIFT) | ADC0CF_AD8BE__NORMAL
			| ADC0CF_ADGN__GAIN_1 | ADC0CF_ADTM__TRACK_NORMAL;

	/*
	 // ADPWR (Burst Mode Power Up Time) = 0x0F
	 // ADLPM (Low Power Mode Enable) = LP_BUFFER_ENABLED (Enable low power
	 //     mode (requires extended tracking time).)
	 // ADMXLP (Mux and Reference Low Power Mode Enable) = LP_MUX_VREF_ENABLED
	 //     (Low power mode enabled (SAR clock < 4 MHz).)
	 // ADBIAS (Bias Power Select) = MODE3 (Select bias current mode 3 (SARCLK
	 //     <= 4 MHz).)
	 */
	ADC0PWR = (0x0F << ADC0PWR_ADPWR__SHIFT) | ADC0PWR_ADLPM__LP_BUFFER_ENABLED
			| ADC0PWR_ADMXLP__LP_MUX_VREF_ENABLED | ADC0PWR_ADBIAS__MODE3;

	/*
	 // ADEN (ADC Enable) = ENABLED (Enable ADC0 (active and ready for data
	 //     conversions).)
	 // ADCM (Start of Conversion Mode Select) = TIMER2 (ADC0 conversion
	 //     initiated on overflow of Timer 2.)
	 */
	ADC0CN0 &= ~ADC0CN0_ADCM__FMASK;
	ADC0CN0 |= ADC0CN0_ADEN__DISABLED | ADC0CN0_ADBMEN__BURST_ENABLED /* ADC0CN0_ADCM__TIMER2 */;
	ADC0AC |= ADC0AC_ADSJST__LEFT_NO_SHIFT;
	REF0CN &= ~REF0CN_REFSL__FMASK;
	REF0CN |= REF0CN_REFSL__VDD_PIN;

	// Enable ADC Interrupt
	EIE1 |= EIE1_EADC0__ENABLED;

	return NULL;
}

Error adc_startConversion( ADCchannel adcChannel )
{
	adcChannel->state = ADC_STATE_SAMPLING;
	currentChannel = adcChannel;

	ADC0CN0 |= ADC0CN0_ADBUSY__SET;

	return NULL;
}

bool adc_isSampled( ADCchannel adc )
{
	return adc->state == ADCSTATE_IDLE;
}

uint16_t adc_getLastValue( ADCchannel adc )
{
	uint16_t result;

	do
	{
		adc->overwritten = false;
		result = adc->lastValue;

	} while( adc->overwritten );

	return result;
}

//-----------------------------------------------------------------------------
// ADC0EOC_ISR
//-----------------------------------------------------------------------------
//
// ADC0EOC ISR Content goes here. Remember to clear flag bits:
// ADC0CN0::ADINT (Conversion Complete Interrupt Flag)
//
//-----------------------------------------------------------------------------
SI_INTERRUPT (ADC0EOC_ISR, ADC0EOC_IRQn)
{
  assert( currentChannel != NULL );

  // copy result
  currentChannel->lastValue = ((uint16_t) ADC0H) << 8 | ADC0L;
  currentChannel->state = ADCSTATE_IDLE;

  currentChannel = NULL;

  // clear interrupt flag
  ADC0CN0 &= ~ADC0CN0_ADINT__SET;
}
