/**
 * \file main.c
 * Main code
 * The based of the project TRACKING SALFORD 
 */

/************************************************************************/
/*						TRACKING PROJECT SALFORD                        */
/************************************************************************/


#include "../lib/main.h"
#include "../lib/tracking.h"
#include "../lib/lcd.h"
/*** WARNING MACRO USE FOR DEBUGGING, WILL BE DELETE ***/
#define DEBUG 0
#define LCD	  0
#define UART  1	
#define PWM   0



// [+]Setup the TIMER1_COMPA interrupt - it will be our tick interrupt.
TASK_ISR(TIMER1_COMPA_vect, tick_interrupt());

volatile char x = ' ';


void my_itoa(int value, BYTE *buf, int base){
	
	int i = 30;
	
	buf = "";
	
	for(; value && i ; --i, value /= base) buf = "0123456789abcdef"[value % base] + buf;
	
}


/*!
 * Main function.
 *
 * Ref adc_read() ...
 */
int main(void)
{
	char x = ' ';
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
	
	#endif
		
	/*** WAITING ***/
	#if DEBUG
	#else
	_delay_ms(2000);
	#endif
	
	#if LCD
	LCD_command(LCD_CLR); 
	#endif
	
			distanceIrRight = readInfrared(ADC_CH_IR_RIGHT);
			distanceIRrLeft = readInfrared(ADC_CH_IR_LEFT);
			//my_itoa(distanceIrRight, lcdBuffer, 10);
			//LCD_write(lcdBuffer);
			//
			//LCD_command(LCD_CLR); 
			//if(thermalDataPtr != NULL)
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

	// [+]The idle task sleeps the CPU  -  set the sleep mode to IDLE,
	// as we need the sleep to be interruptable by the tick interrupt.
	set_sleep_mode(SLEEP_MODE_IDLE);
			//
			///*** TEST IR SENSOR ***/
			//distanceIrRight = lookupInfrared(adcResultCh0);
			//
			///*** TEST ADC CHANNEL 1 ***/
			//adcResultCh1 = adc_read(ADC_CH_IR_LEFT);
					//
			///*** TEST IR SENSOR ***/
			//distanceIRrLeft = lookupInfrared(adcResultCh1);
		#if UART
		printf("\n%d;%d", distanceIrRight, distanceIRrLeft);
		#endif

	// [+]Create tasks.
	/*** WARNING !!  Priority and Buffer NEED TO BE VERIFY ***/
	create_task(taskSensor, 0, 0, 100U, 100U, 0);
	create_task(taskSerialTx, 0, 0, 100U, 60U, 0);
	create_task(taskSerialRx, 0, 0, 100U, 60U,  0);
	create_task(taskTracking, 0, 0, 100U, 50U,  0);
		_delay_ms(100);
		#endif
		
		#if LCD
		LCD_command(LCD_CLR);
		#endif 

	init_timer(1000U);	//!< \Set TIMER1_COMPA interrupt to tick every 80,000 clock cycles.
		
		_delay_ms(500);
		#endif

	// [+]Start the RTOS - note that this function will never return.
	task_switcher_start(idle_task, 0, 65U, 80U);
		//tracking(distanceIrRight, distanceIRrLeft);
	
	
	/*** NO INFINITE LOOP IN MAIN : RTOS RUN ***/
		 	 
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
	
	printf("\n~ Board Ready ~\n");
	
	/*** END OF INIT PART ***/
}


void delay_ms(unsigned int t)
{
	wait_for_increment_of(&tick, t, 0, 0);
}


void taskSensor(void *p)
{
	while(1)
	{
		thermalDataPtr = mesure_thermal(thermal_Buff, THERMAL_BUFF_SIZE - 1) ;			//<! \Mesure of the thermal
		
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
		
		printf("\nt1");
		delay_ms(DELAY_TSENSOR);
	}
}


void taskSerialTx(void *p)
{
	while(1)
	{
		/*** TEST THERMAL SENSOR ***/
		if(thermalDataPtr != NULL)
		{
			for (index = 0 ; index < THERMAL_TP_SIZE-1 ; index++)
			{
			printf(" %d", thermalDataPtr[index]);
			}
			printf("\r\n");
		}
		else
		{
			return(-1);
			printf("\r\n thermal error ...");
		}
		
		//printf("\r\n%d\t%d\t%d\t%d",adcResultCh0, adcResultCh1, distanceIrRight, distanceIRrLeft);
		
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
				
		printf("\nt2");
		delay_ms(DELAY_TSERIALTX);
	}
}


void taskSerialRx(void *p)
{
	while(1)
	{
		//while(x = uart_getchar())
		//{
			//if(x == 'o')
			//{
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
						//
				//printf("%d;%d;%d;%d\n" ,adcResultCh0, distanceIrRight, adcResultCh1, distanceIRrLeft);
						//
				//x = ' ';
						//
				////printf("\n\rLog value");
			//}
		//
		//
		//}
		
		printf("\nt3");
		delay_ms(DELAY_TSERIALRX);	
	}
}


void taskTracking(void *p)
{
		
	while(1)
	{
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
		
		//if(x == 'o')
		//{
			//if (flagSensorValueChanged == 1)
			//{
				//flagSensorValueChanged = 0;
				//printf("%d;%d\n",distanceIRrLeft, distanceIrRight);
			//}
			//
			//x = ' ';
			//
			//printf("\n\rLog value");
		//}
		//
		//else if (x == 's')
		//{			
			//printf("\n\rStop log ir ...");
			//
			//x = ' ';
		//}

		printf("\nt4");		
		delay_ms(DELAY_TTRACKING);
	}	
}


void init_timer(unsigned int hz)
{
	// [+]Set TIMER1_COMPA interrupt to tick every 80,000 clock cycles.
	TCCR1B = (1<<WGM12) && (1<<CS11); // CTC, clkI/O/8 (From prescaler).
	OCR1A  = (F_CPU / (8 * hz)) - 1;  // Formula: (f_cpu / (64 * desired_f)) - 1, ex: f_pwm=1000Hz (period = 1ms standard).
	TIFR   = (1<<OCF1A);
	TIMSK |= (1<<OCIE1A);
}


// This is our idle task - the task that runs when all others are suspended.
// We sleep the CPU - the CPU will automatically awake when the tick interrupt occurs
// This task cannot be stopped, so it is automatically re-started whenever it tries to exit.
void idle_task(void *p)
{
	sleep_enable();
	sei();
	sleep_cpu();
}


// This is a function that runs every tick interrupt - we use it to increment the tick semaphore value by one
// We want a task switch to ALWAYS occur - it is part of the definition of the tick interrupt!
uint8_t tick_interrupt()
{
	increment_semaphore_by(&tick, 1);
	return(1);
}
