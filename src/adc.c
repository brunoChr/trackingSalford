/*
 * \file adc.c
 *
 * Created: 03/10/2015 10:22:35
 *  Author: b.christol
 */ 

#include "../lib/port.h"
#include "../lib/adc.h"

/* TODO

*/

/*! \fn		void adc_init()
 *  \brief	Init adc module
 */
void adc_init()
{
	/*
		REFS1 REFS0 Voltage Reference Selection
		0		0	AREF, Internal Vref turned off
		0		1	AVCC with external capacitor at AREF pin
		1		0	Reserved
		1		1	Internal 2.56V Voltage Reference with external capacitor at AREF pin
	*/
	
	// AREF = AVcc
	ADMUX = (1<<REFS0); // Set bits REFS0 : AVCC with external capacitor at AREF pin
	ADMUX = (1<<REFS1);
	
	// ADC Enable and prescaler of 128
	// F_CPU/128 = ...
	/*	ADEN : ADC ENABLE
		ADPSx : prescaler 
		ADPS0.1.2 = 111 -> prescaler at 128
	*/
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	
}


/*! \fn		unsigned int adc_read(BYTE ch)
 *  \brief	Get ADC raw value on a channel
 *  \param	ch: the channel 
 *  \return	The raw value of the ADC
 */
unsigned int adc_read(BYTE ch)
{
	// select the corresponding channel 0~7
	// ANDing with ’7? will always keep the value
	// of ‘ch’ between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
	
	// start single convertion
	// write ’1? to ADSC
	ADCSRA |= (1<<ADSC);
	
	// wait for conversion to complete
	// ADSC becomes ’0? again
	// till then, run loop continuously
	// TODO : maybe use interrupt ????
	while(ADCSRA & (1<<ADSC));
	
	return (ADC);
}


/*! \fn		FLOAT adc2MilliVolt(unsigned int adcData)
 *  \brief	Convert ADC raw value in mV
 *  \param	adcData: adc raw value
 *  \return	The value in mV
 */
FLOAT adc2MilliVolt(unsigned int adcData)
{
	return ((adcData*5)/1023);
}
