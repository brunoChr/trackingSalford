/************************************************************************/
/*						TRACKING PROJECT SALFORD                        */
/************************************************************************/


#include "../lib/cpu.h"
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "../lib/twi.h"
#include "../lib/port.h"
#include "../lib/uart.h"
#include "../lib/types.h"
#include "../lib/thermal.h"
#include "../lib/generics.h"
#include "../lib/adc.h"
#include "../lib/pwm.h"
#include "../lib/infrared.h"
#include "../lib/serialData.h"

#define TAILLE_DATA 4*4
#define ADC_CH_IR_RIGHT	0
#define ADC_CH_IR_LEFT	1

//Globalvar.
BYTE thermal_data[THERMAL_BUFF_SIZE];
BYTE* randomByteArray();

void setup(void)
{
	/*** INIT ***/

	port_init();
	uart_init(9600);
	//servo_init();
	adc_init();
	pwm_init();
	//LCD_init();
	if(!twi_init(100000)) // Init I2C  with 100KHz bitrate.
	{
		printf("\nError in initiating I2C interface.");
		while(1);
	}
	/*** END OF INIT PART ***/	
}


/*!
 * Main function.
 *
 * Ref adc_read(), sharp_IR_interpret_GP2Y0A02YK(), pwm_positionCentrale().
 */
int main(void)
{
	//BYTE *thermalData;
	uint16_t adcResultCh0, adcResultCh1;
	int8_t  distanceIRrLeft, distanceIrRight;
	BYTE *dataSerial;
	
	/*** VARIABLE INITIALISATION ***/
	adcResultCh0 = 0;
	adcResultCh1 = 0;
	distanceIrRight = 0;
	distanceIRrLeft = 0;
	
	//_delay_ms(1000);
		
	/*** SETUP SYSTEM ***/
	setup();
	
	//printf("\n~ Board Ready ~\n");
	
	/*** WAITING ***/
	//_delay_ms(1000);

	formatProtocol(IR_R_SENSOR, randomByteArray());

	/*** INFINITE LOOP ***/
	while(1)
	{	
		
		/*** TEST THERMAL SENSOR ***/		
		//mesure_thermal(thermal_data, THERMAL_BUFF_SIZE-1);
	
		//printf("\n\rthermalData : ");
	
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
		
		/*** TEST ADC CHANNEL 0 ***/
		adcResultCh0 = adc_read(ADC_CH_IR_RIGHT);
		
		/*** TEST IR SENSOR ***/
		distanceIrRight = sharp_IR_interpret_GP2Y0A02YK(adcResultCh0);
		
		
		/*** TEST ADC CHANNEL 1 ***/
		adcResultCh1 = adc_read(ADC_CH_IR_LEFT);
				
		/*** TEST IR SENSOR ***/
		distanceIRrLeft = sharp_IR_interpret_GP2Y0A02YK(adcResultCh1);
		
		}
		
		/* PRINT ADC VALUES */
		printf("%d\t%d\t%d\t%d\r",adcResultCh0, adcResultCh1, distanceIrRight, distanceIRrLeft);
		//printf("%d\t%d\r",distanceIrRight, distanceIRrLeft);
		//uart_putchar(distanceIrRight);
		//uart_putchar(distanceIRrLeft);
		
		
		/*** TEST PWM SERVO ***/
		pwm_positionCentrale();
		
		//_delay_ms(5000);
		
		//cli();
			
 		}
		 
	 
	return(0);
}



ISR(TIMER1_OVF_vect)
{
	/*
	* Routine d'interruption activee lors d'un
	* overflow du timer1
	*/
	
	PORTB |= (1 <<PORTB5);
}

ISR(TIMER1_COMPA_vect)
{
	/*
	* Routine d'interruption activee lors du compare match
	* entre ocr1 (temps à l'état haut) et timer1
	*/
	PORTB|= (0 << PORTB5);
}


BYTE* randomByteArray()
{
	int size = 8;
	int i;
	BYTE array[size];
	BYTE *aPtr = malloc(sizeof(BYTE) * size);
	
	srand(1);
	
	for (i = 0; i < size; i++)
	{
		aPtr[i] = (rand() % 101) + 500;
	}
	
	return *aPtr;
}