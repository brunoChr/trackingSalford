/************************************************************************/
/*						TRACKING PROJECT SALFORD                        */
/************************************************************************/


/************************************************************************/
/* KEYBOARD SHORTCUT 
 * CommentSelection		CTRL+K, CTRL+C
 * UncommentSelection	CTRL+K, CTRL+U                          
 * TabLeft				SHIFT+TAB
 * 
*/
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


#define TAILLE_DATA 4*4		//!< \Matrix temp sensor
#define ADC_CH_IR_RIGHT	0	//!< \ADC channel of the right IR sensor
#define ADC_CH_IR_LEFT	1	//!< \ADC channel of the left IR sensor

#define DEBUG 0


/*** Globalvar ***/

BYTE thermal_Buff[THERMAL_BUFF_SIZE];	//!< \Buffer of temp
BYTE *thermalDataPtr;					//!< \Pointer to the buffer temp


/*** Prototype function main ***/

BYTE * getRandom();		//!< \Return a 16 byte array fill with random number
void setup(void);		//!< \Init function of the system


/*!
 * Main function.
 *
 * Ref adc_read(), sharp_IR_interpret_GP2Y0A02YK(), pwm_positionCentrale().
 */
int main(void)
{
	uint16_t adcResultCh0, adcResultCh1;
	int8_t  distanceIRrLeft, distanceIrRight;
	BYTE *dataSerial;
	serialProtocol Frame;
	BYTE index;
	SHORT pos;
	
	/*** VARIABLE INITIALISATION ***/
	adcResultCh0 = 0;
	adcResultCh1 = 0;
	distanceIrRight = 0;
	distanceIRrLeft = 0;
	pos = -90;
	
	//_delay_ms(1000);
		
	/*** SETUP SYSTEM ***/
	setup();
	
	printf("\n~ Board Ready ~\n");
	
	/*** WAITING ***/
	#if DEBUG
	#else
	_delay_ms(1000);
	#endif


	/*** INFINITE LOOP ***/
	while(1)
	{	
		
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
		
		thermalDataPtr = mesure_thermal(thermal_Buff, THERMAL_BUFF_SIZE - 1) ;
			
		/*** TEST THERMAL SENSOR ***/
		if(thermalDataPtr != NULL)
		{
			//for (index = 0 ; index < THERMAL_TP_SIZE-1 ; index++)
			//{
				//printf(" %d", thermalDataPtr[index]);
			//}		
			//printf("\r\n");
		}
		
		else
		{
			printf("\r\n thermal error ...");
		}
		
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
		//printf("\r\n%d %f",adcResultCh0, adc2MilliVolt(adcResultCh0));
		//printf("\r\n%d\t%d\t%d\t%d",adcResultCh0, adcResultCh1, distanceIrRight, distanceIRrLeft);
		//printf("%d\t%d\r",distanceIrRight, distanceIRrLeft);
		//uart_putchar(distanceIrRight);
		//uart_putchar(distanceIRrLeft);
	
		/*** TEST FORMAT PROTOCOL ***/
		Frame = formatProtocol(THERMAL_SENSOR, thermalDataPtr, NBR_DATA);
		
		printf("\r\n%d%d", Frame.sb, Frame.id);
		
		for (index = 0; index < NBR_DATA; index++)
		{
			printf("%d", Frame.data[index]);
		}
		
		printf("%d%d%d", Frame.cs, Frame.cn, Frame.eb);
		
		
		///*** TEST PWM SERVO ***/
		
		//if(pos < 80)
		//{
			//pwm_setPosition((pos));
			//pos += 1;	
		//}
		//else
		//{
			//pos = -90;	
		//}
		
		#if DEBUG
		#else
		_delay_ms(1000);
		#endif
		
		//
		///*** TEST PWM SERVO ***/
		//pwm_setPosition(90);
		//
		//_delay_ms(5000);
		//
		///*** TEST PWM SERVO ***/
		//pwm_setPosition(180);
		//
		//_delay_ms(5000);
		//
		//
		//pwm_setPosition(0);
		
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
	
	PORTB |= (1 << PB5);
}

ISR(TIMER1_COMPA_vect)
{
	/*
	* Routine d'interruption activee lors du compare match
	* entre ocr1 (temps à l'état haut) et timer1
	*/
	
	PORTB |= (0 << PB5);
}


/* function to generate and return random numbers */
BYTE * getRandom( )
{
	static BYTE  r[16];
	int i;

	/* set the seed */
	srand(10);
	for ( i = 0; i < 10; ++i)
	{
		r[i] = rand();
		//printf( "r[%d] = %d\n", i, r[i]);
	}

	return r;
}

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
