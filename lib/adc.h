/*
 * adc.h
 *
 * Created: 03/10/2015 15:12:26
 *  Author: b.christol
 */ 


#ifndef ADC_H_
#define ADC_H_

#include "types.h"

#define ADC_VREF 5.0					//<! \Full scale voltage ! 5V
#define ADC_RES_BIT 10					//<! \Number of resolution bits of ADC
#define ADC_RES		((2^10) - 1)		//<! \Number of bit of ADC


void adc_init();
unsigned int adc_read(BYTE ch);
FLOAT adc2MilliVolt(unsigned int adcData);

#endif /* ADC_H_ */