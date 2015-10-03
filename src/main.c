/************************************************************************/
/*						TRACKING PROJECT SALFORD                        */
/************************************************************************/


#include "../lib/cpu.h"
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "../lib/twi.h"
#include "../lib/port.h"
#include "../lib/uart.h"
#include "../lib/types.h"
#include "../lib/thermal.h"
#include "../lib/generics.h"

#define TAILLE_DATA 4*4
#define ADC_CH_IR_RIGHT	0
#define ADC_CH_IR_LEFT	1


void setup(void)
{
	/*** INIT ***/

	port_init();
	uart_init(9600);
	//servo_init();
	adc_init();
	//LCD_init();
	if(!twi_init(100000)) // Init I2C  with 100KHz bitrate.
	{
		printf("\nError in initiating I2C interface.");
		while(1);
	}
	/*** END OF INIT PART ***/	
}

int main(void)
{
	BYTE randData;
	BYTE *thermalData;
	BYTE index;
	uint16_t adcResult;
	int8_t  distanceIRrLeft, distanceIrRight;
	
	
	/*** VARIABLE INITIALISATION ***/
	randData = 0;
	index = 0;
	adcResult = 0;
	distanceIrRight = 0;
	distanceIRrLeft = 0;
	
	/*** SETUP SYSTEM ***/
	setup();
	
	printf("\n~ Board Ready ~\n");
	
	/*** WAITING ***/
	_delay_ms(1000);


	/*** INFINITE LOOP ***/
	while(1)
	{	
		
		/*** TEST THERMAL SENSOR ***/		
		thermal_read(0x14, thermalData);
	
		_delay_ms(1000);	
	
		//printf("\n\rthermalData : ");
	
		for (index = 0; index < THERMAL_BUFF_SIZE; index++ )
		{
			uart_putchar(thermalData[index]);
		}
				
		/*** TEST ADC CHANNEL 0 ***/
		adcResult = adc_read(ADC_CH_IR_RIGHT);
		
		/*** TEST IR SENSOR ***/
		distanceIrRight = sharp_IR_interpret_GP2Y0A02YK(adcResult);
		
		
		/*** TEST ADC CHANNEL 1 ***/
		//adcResult = 0;
		adcResult = adc_read(ADC_CH_IR_LEFT);
				
		/*** TEST IR SENSOR ***/
		distanceIRrLeft = sharp_IR_interpret_GP2Y0A02YK(adcResult);
		
		/* PRINT */
		uart_putchar(distanceIrRight);
		uart_putchar(distanceIRrLeft);
 		}
		 
	 
	return(0);
}
