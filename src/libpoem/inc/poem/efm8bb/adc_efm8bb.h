#ifndef ADC_EFM8BB_H
#define ADC_EFM8BB_H

typedef enum { ADCSTATE_IDLE, ADC_STATE_SAMPLING } ADCState;

typedef struct sADCchannel
{
  int pin;
  ADCState state;
  uint16_t lastValue;
  // overwritten is cleared before reading the ADC value.
  // if it is still cleared after reading the value the value was valid
  // else it was overwritten, and should be re-read.
  int overwritten;
} sADCchannel, *ADCchannel;

Error efm8bb_adc_create_burst( ADCchannel adcChannel, int pin, int resolution_bits
		                       /* , uint16_t ksps */ );

Error adc_startConversion( ADCchannel adcChannel );
bool adc_isSampled( ADCchannel adc );
uint16_t adc_getLastValue( ADCchannel adc );

#endif
