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



/*** GLOBAL VARIABLE, MOVE TO .h ***/

static BYTE thermal_Buff[THERMAL_BUFF_SIZE];		//!< \Buffer of temp
static BYTE *thermalDataPtr;						//!< \Pointer to the buffer temp
static semaphore_t tick = {0};						//!< \A semaphore is incremented at every tick.
static UINT  distanceIRrLeft, distanceIrRight;
static SHORT pos;
flagReceive flagRx;

BOOL flagSensorValueChanged;
//BOOL flagReceiveValue;
BYTE bufferSerialRx[32];

/*** Prototype function main ***/

static BYTE * getRandom();							//!< \Return a 16 byte array fill with random number
static void setup(void);							//!< \Init function of the system
void delay_ms(unsigned int t);				//!< \Wait ms

void init_timer(unsigned int hz);
void idle_task(void *p);
uint8_t tick_interrupt();


/*** WARNING ! MAYBE TURN INTO STATIC ***/
void taskSensor(void *p);					//!< \Task update of sensor
void taskSerialTxRx(void *p);					//!< \Task serial communication emission
void taskSerialCmd(void *p);					//!< \Task serial communication reception
void taskTracking(void *p);					//!< \Task tracking

// [+]Setup the TIMER1_COMPA interrupt - it will be our tick interrupt.
TASK_ISR(TIMER1_COMPA_vect, tick_interrupt());



/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
void my_itoa(int value, BYTE *buf, int base){
	
	int i = 30;
	
	buf = (BYTE *)"";
	
	for(; value && i ; --i, value /= base) (buf = "0123456789abcdef"[value % base] + buf);
	
}


/*!
 * Main function.
 *
 * Ref adc_read() ...
 */
int main(void)
{
	//BYTE lcdBuffer[24];

	/*** VARIABLE INITIALISATION ***/
	

	/*** SETUP SYSTEM ***/
	setup();				
	
	cli();			 // Interrupts should remain disabled - they will be enabled as soon as the first task starts executing.
	clr(B,0);
	
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
	

	// [+]The idle task sleeps the CPU  -  set the sleep mode to IDLE,
	// as we need the sleep to be interruptable by the tick interrupt.
	set_sleep_mode(SLEEP_MODE_IDLE);

	#if UART
	//printf("\n%d;%d", distanceIrRight, distanceIRrLeft);
	#endif

	// [+]Create tasks.
	/*** WARNING !!  Priority and Buffer NEED TO BE VERIFY ***/
	create_task(taskSensor, 0, 0, 65U, 100U, 0);		//<! \Size of stack & Priority
	create_task(taskSerialTxRx, 0, 0, 80U, 60U, 0);
	//create_task(taskSerialCmd, 0, 0, 100U, 60U,  0);
	//create_task(taskTracking, 0, 0, 100U, 40U,  0);
	

	init_timer(5000U);	//!< \Set TIMER1_COMPA interrupt to tick every 80,000 clock cycles.

	// [+]Start the RTOS - note that this function will never return.
	task_switcher_start(idle_task, 0, 65U, 80U);
	
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



/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
/* function to generate and return random numbers */
static BYTE * getRandom( )
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


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
static void setup(void)
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


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
void delay_ms(unsigned int t)
{
	wait_for_increment_of(&tick, t, 0, 0);
}


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
void taskSensor(void *p)
{
	while(1)	
	{
		//printf("\nUart : %d", uart_kbhit());
		//uart_putchar('a');
		thermalDataPtr = mesure_thermal(thermal_Buff, THERMAL_BUFF_SIZE - 1) ;			//<! \Mesure of the thermal
		
		distanceIrRight = readInfrared(ADC_CH_IR_RIGHT);
		distanceIRrLeft = readInfrared(ADC_CH_IR_LEFT);
		//my_itoa(distanceIrRight, lcdBuffer, 10);
		//LCD_write(lcdBuffer);
		//
		//LCD_command(LCD_CLR);
		
		flagSensorValueChanged = 1;
		
		//printf("\ntSensor");
		delay_ms(DELAY_TSENSOR);
	}
}


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
volatile char rxData = ' ';

void taskSerialTxRx(void *p)
{
	while(1)
	{	
		//printf("\ntRxTx");
		
		if((rxData = uart_getchar()))
		{
			if(rxData == CMD_START)
			{	
				flagRx.start = 1;
				
				#if VERBOSE
				uart_putchar('\n');
				uart_putchar('C');
				#endif 
			}
			
			rxData = ' ';
		}
		
		if(flagRx.start)
		{
			//printf("\r\nStart");
			
			//rxData = ' ';
			if ((rxData = uart_getchar()))
			{
				if (rxData == CMD_IRL)
				{
					#if VERBOSE
					uart_putchar('L');
					uart_putchar('\n');
					#endif
					
					/*** TEST RIR ***/
					
					sendFrame((BYTE *)distanceIRrLeft, 2);
					
					
					flagRx.start = 0;
				}
				else if(rxData == CMD_IRR)
				{
					//printf("\r\nCMD IRR");
					#if VERBOSE
					uart_putchar('R');
					uart_putchar('\n');
					#endif
					
					/*** TEST RIR ***/
					sendFrame((BYTE *)distanceIrRight, 2);
					
					
					flagRx.start = 0;
				}
				else if(rxData == CMD_THERM)
				{
					//printf("\r\nCMD THERM");
					#if VERBOSE
					uart_putchar('T');
					uart_putchar('\n');
					#endif
					
					/*** TEST THERMAL SENSOR ***/
					sendFrame(thermalDataPtr, NBR_DATA_THERM);
										
					flagRx.start = 0;
				}
				else if(rxData == CMD_SERVO)
				{
					//printf("\r\nCMD SERVO");
					#if VERBOSE
					uart_putchar('S');
					uart_putchar('\n');
					#endif
					
					/*** TEST SEND SERVO ***/
					flagRx.start = 0;
				}
				else
				{
					#if VERBOSE
					uart_putchar('U');
					uart_putchar('\n');
					#endif
					//printf("\r\nCMD UNKNOWN");
					flagRx.start = 0;
				}
				
				rxData = ' ';
			}
		}
		
		
		delay_ms(DELAY_TSERIALTX);
	}
}


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
void taskSerialCmd(void *p)
{
	while(1)
	{
		//while(rxData = uart_getchar())
		//{
			//if(rxData == 'o')
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
				//rxData = ' ';
						//
				////printf("\n\rLog value");
			//}
		//
		//
		//}
		
		//printf("\ntCmd");
		delay_ms(DELAY_TSERIALRX);	
	}
}


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
void taskTracking(void *p)
{
	pos = 90;
	int newPos = 0;
	
	while(1)
	{
		/*
		newPos = tracking(pos);
		pos = newPos;
		*/
		pwm_setPosition(10);
		_delay_ms(200);
		pwm_setPosition(90);
		_delay_ms(200);
		pwm_setPosition(180);
		_delay_ms(200);
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
		
		//if(rxData == 'o')
		//{
			//if (flagSensorValueChanged == 1)
			//{
				//flagSensorValueChanged = 0;
				//printf("%d;%d\n",distanceIRrLeft, distanceIrRight);
			//}
			//
			//rxData = ' ';
			//
			//printf("\n\rLog value");
		//}
		//
		//else if (rxData == 's')
		//{			
			//printf("\n\rStop log ir ...");
			//
			//rxData = ' ';
		//}

		//printf("\ntTrack");		
		delay_ms(DELAY_TTRACKING);
	}	
}



/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
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
/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
void idle_task(void *p)
{
	sleep_enable();
	sei();
	sleep_cpu();
}


// This is a function that runs every tick interrupt - we use it to increment the tick semaphore value by one
// We want a task switch to ALWAYS occur - it is part of the definition of the tick interrupt!
/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
uint8_t tick_interrupt()
{
	increment_semaphore_by(&tick, 1);
	return(1);
}
