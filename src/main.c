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
#include "../lib/tracking.h"
#include "../lib/lcd.h"


#define TAILLE_DATA 4*4		//!< \Matrix temp sensor
#define ADC_CH_IR_RIGHT	0	//!< \ADC channel of the right IR sensor
#define ADC_CH_IR_LEFT	1	//!< \ADC channel of the left IR sensor


/*** WARNING MACRO USE FOR DEBUGGING, WILL BE DELETE ***/
#define DEBUG 0
#define LCD	  0
#define UART  1	
#define PWM   0

/*** Globalvar ***/

BYTE thermal_Buff[THERMAL_BUFF_SIZE];	//!< \Buffer of temp
BYTE *thermalDataPtr;					//!< \Pointer to the buffer temp


/*** Prototype function main ***/

BYTE * getRandom();		//!< \Return a 16 byte array fill with random number
void setup(void);		//!< \Init function of the system


void my_itoa(int value, BYTE *buf, int base){
	
	int i = 30;
	
	buf = "";
	
	for(; value && i ; --i, value /= base) buf = "0123456789abcdef"[value % base] + buf;
	
}


/*!
 * Main function.
 *
 * Ref adc_read(), sharp_IR_interpret_GP2Y0A02YK(), pwm_positionCentrale().
 */
int main(void)
{
	UINT adcResultCh0, adcResultCh1;
	UINT  distanceIRrLeft, distanceIrRight;
	BYTE *dataSerial;
	serialProtocol Frame;
	BYTE index;
	BYTE lcdBuffer[24];
	//SHORT pos;
	
	/*** VARIABLE INITIALISATION ***/
	adcResultCh0 = 0;
	adcResultCh1 = 0;
	distanceIrRight = 0;
	distanceIRrLeft = 0;
	
	
	clr(B,0);
		
	/*** SETUP SYSTEM ***/
	setup();
	
	#if UART
	printf("\n~ Board Ready ~\n");
	#endif
	
	/*** WAITING ***/
	#if DEBUG
	#else
	_delay_ms(2000);
	#endif
	
	#if LCD
	LCD_command(LCD_CLR); 
	#endif
	
	/*** INFINITE LOOP ***/
	while(1)
	{	
		
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			distanceIrRight = readInfrared(ADC_CH_IR_RIGHT);
			distanceIRrLeft = readInfrared(ADC_CH_IR_LEFT);
			
			//LED(distanceIrRight % 256);
			//my_itoa(distanceIrRight, lcdBuffer, 10);
			//LCD_write(lcdBuffer);
			//
			//LCD_command(LCD_CLR); 
			
			//
			//thermalDataPtr = mesure_thermal(thermal_Buff, THERMAL_BUFF_SIZE - 1) ;
				//
			///*** TEST THERMAL SENSOR ***/
			//if(thermalDataPtr != NULL)
			//{
				////for (index = 0 ; index < THERMAL_TP_SIZE-1 ; index++)
				////{
					////printf(" %d", thermalDataPtr[index]);
				////}		
				////printf("\r\n");
			//}
			//
			//else
			//{
				//return(-1);
				////printf("\r\n thermal error ...");
			//}
		
			///*** TEST ADC CHANNEL 0 ***/
			//adcResultCh0 = adc_read(ADC_CH_IR_RIGHT);
			//
			///*** TEST IR SENSOR ***/
			//distanceIrRight = lookupInfrared(adcResultCh0);
			//
			///*** TEST ADC CHANNEL 1 ***/
			//adcResultCh1 = adc_read(ADC_CH_IR_LEFT);
					//
			///*** TEST IR SENSOR ***/
			//distanceIRrLeft = lookupInfrared(adcResultCh1);
		
		}
		
		#if UART
		printf("\n%d;%d", distanceIrRight, distanceIRrLeft);
		#endif
					
		/* PRINT ADC VALUES */
		//printf("\r\n%d %f",adcResultCh0, adc2MilliVolt(adcResultCh0));
		//printf("\r\n%d\t%d\t%d\t%d",adcResultCh0, adcResultCh1, distanceIrRight, distanceIRrLeft);
		//printf("%d\t%d\r",distanceIrRight, distanceIRrLeft);
		//uart_putchar(distanceIrRight);
		//uart_putchar(distanceIRrLeft);
	
		/*** TEST FORMAT PROTOCOL ***/
		//Frame = formatProtocol(THERMAL_SENSOR, thermalDataPtr, NBR_DATA);
		//
		//uart_putchar(Frame.sb);
		//uart_putchar(Frame.id);
		//
		//for (index = 0; index < NBR_DATA; index++)
		//{
			//uart_putchar(Frame.data[index]);
		//}
		//
		//uart_putchar(Frame.cs);
		//uart_putchar(Frame.cn);
		//uart_putchar(Frame.eb);


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
		_delay_ms(100);
		#endif
		
		#if LCD
		LCD_command(LCD_CLR);
		#endif 
		
		
		
		/*** TEST PWM SERVO ***/
		#if PWM
		pwm_setPosition(45);
		_delay_ms(500);
		pwm_setPosition(90);
		_delay_ms(500);
		pwm_setPosition(115);
		_delay_ms(500);
		pwm_setPosition(150);
		_delay_ms(500);
		pwm_setPosition(10);
		_delay_ms(500);
		#endif
		
		//cli();
		/* Test Tracking */
		//tracking(distanceIrRight, distanceIRrLeft);
		
 		}
		 
	return(0);
}


ISR(TIMER3_OVF_vect)
{
	/*
	* Routine d'interruption activee lors d'un
	* overflow du timer1
	*/
	
	PORTE |= (1 <<PE3);
}

ISR(TIMER3_COMPA_vect)
{
	/*
	* Routine d'interruption activee lors du compare match
	* entre ocr1 (temps à l'état haut) et timer1
	*/
	PORTE|= (0 << PE3);
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
	
	#if PWM
	pwm_init();
	#endif
	
	#if LCD
	LCD_init();
	LCD_write("Tracking");
	LCD_command(LCD_LINE2 | 0); // move cursor  to row 2, position 5
	LCD_write("Team WTF");
	#endif
	
	if(!twi_init(100000)) // Init I2C  with 100KHz bitrate.
	{
		printf("\nError in initiating I2C interface.");
		while(1);
	}
	/*** END OF INIT PART ***/
}
