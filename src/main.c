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


/*** GLOBAL VARIABLE, MOVE TO .h ***/
volatile char x = ' ';


enum ReceiveCmd
{
	CMD_START = '#',
	CMD_IRR = '1',
	CMD_IRL = '2',
	CMD_THERM = '3',
	CMD_SERVO = '4',	
};


typedef struct flagReceive
{
	BOOL start;
	
} flagReceive;


flagReceive flagRx;
//enum ReceiveCmd RxCmd;

BOOL flagSensorValueChanged;
//BOOL flagReceiveValue;
BYTE bufferSerialRx[32];

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

	/*** VARIABLE INITIALISATION ***/
	adcResultCh0 = 0;
	adcResultCh1 = 0;
	distanceIrRight = 0;
	distanceIRrLeft = 0;
	

	/*** SETUP SYSTEM ***/
	setup();				
	
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
	create_task(taskSensor, 0, 0, 400U, 100U, 0);		//<! \Size of stack & Priority
	create_task(taskSerialTxRx, 0, 0, 400U, 60U, 0);
	create_task(taskSerialCmd, 0, 0, 400U, 60U,  0);
	create_task(taskTracking, 0, 0, 400U, 40U,  0);
	
	#if LCD
	LCD_command(LCD_CLR);
	#endif

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


void delay_ms(unsigned int t)
{
	wait_for_increment_of(&tick, t, 0, 0);
}


void taskSensor(void *p)
{
	while(1)	
	{
		thermalDataPtr = mesure_thermal(thermal_Buff, THERMAL_BUFF_SIZE - 1) ;			//<! \Mesure of the thermal
		distanceIrRight = readInfrared(ADC_CH_IR_RIGHT);
		distanceIRrLeft = readInfrared(ADC_CH_IR_LEFT);
		//my_itoa(distanceIrRight, lcdBuffer, 10);
		//LCD_write(lcdBuffer);
		//
		//LCD_command(LCD_CLR);
		
		flagSensorValueChanged = 0;
		
		//printf("\ntSensor");
		delay_ms(DELAY_TSENSOR);
	}
}


void taskSerialTxRx(void *p)
{
	while(1)
	{	
				
		if(flagSensorValueChanged == 1)
		{			
			/*** TEST THERMAL SENSOR ***/
			if(thermalDataPtr != NULL)
			{
				/*** WARNING ! PRINTF JUST TO SEE IN PUTTY, WILL BE CHANGED TO uart_putchar ***/
				printf("\r\n");
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
			
			flagSensorValueChanged = 0;	
		}
		
		if(x = uart_getchar())
		{
			if(x == CMD_START)
			{	
				flagRx.start = 1;
				printf("\r\nCmd Start");
			}
			
			x = ' ';
		}
		
		if(flagRx.start)
		{
			printf("\r\nStart");
			
			//x = ' ';
			if (x = uart_getchar())
			{
				if (x == CMD_IRL)
				{
					uart_putchar()
					printf("\r\nCMD IRL");
					flagRx.start = 0;
					x = ' ';
				}
				else if(x == CMD_IRR)
				{
					printf("\r\nCMD IRR");
					flagRx.start = 0;
					x = ' ';
				}
				else if(x == CMD_THERM)
				{
					printf("\r\nCMD THERM");
					flagRx.start = 0;
					x = ' ';
				}
				else if(x == CMD_SERVO)
				{
					printf("\r\nCMD SERVO");
					flagRx.start = 0;
					x = ' ';
				}
			}
		}
		
		printf("\ntRxTx");
		delay_ms(DELAY_TSERIALTX);
	}
}


void taskSerialCmd(void *p)
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
		
		//printf("\ntCmd");
		delay_ms(DELAY_TSERIALRX);	
	}
}


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

		printf("\ntTrack");		
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
