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
#include "../lib/servo.h"
#include "../lib/kalmanFilter.h"
#include "../lib/fusion.h"
#include <math.h>

/*** LOCAL VARIABLE, MOVE TO .h ***/
static BYTE thermal_Buff[THERMAL_BUFF_SIZE];					//!< \Buffer of temp, static because only use in main.c
static INT *thermalDataPtr;										//!< \Pointer to the buffer temp
static double *thermNorm;	
static semaphore_t tick = {0};									//!< \A semaphore is incremented at every tick.
static volatile UINT  distanceIrLeft, distanceIrRight;			//!< \Distance from the sensor : static because only use in main, volatile because modified by different process
volatile const UINT *ptrDistL = &distanceIrLeft;				//!< \Pointer to distance adress because volatile UINT *  means "pointer to volatile UINT", const to be non modifiable	
volatile const UINT *ptrDistR = &distanceIrRight;				//!< \Same
/*** HELP ON POINTER ***/
/* value of distance UINT : distanceIRrLeft, distanceIrRight
 * ptr to volatile UINT : *ptrDistL, *ptrDistR
 * address of distance :  &distanceIRrLeft, &distanceIrRight
 * example : *ptrDistL = &distanceIRrLeft = 0x183 on declaration; in the code *(ptrDistL) = *(&distanceIRrLeft) = distanceIRrLeft = 523cm 
*/

static double distIrLeftNorm, distIrRightNorm;
static flagReceive flagRx;				//<! \Maybe structure is useless to see !!! WARNING !!
static compteur cpt;					//<! \Struct use for counters : sensor ir, therm, timeout ...
static BOOL flagSensorValueChanged;		//<! \Flag is set when data are ready
static volatile UINT state;
static mailbox_t mb_dataIrLeft, mb_dataIrRight;		//<! \data queue to pass data between task
static volatile INT currentDiffMean;
static BYTE prevDiffMean;
static servo servoT;
static thermal thermT;
static kalman_state kalmanIrR;		//<! \Structure for kalman filter for right IR	
static kalman_state kalmanIrL;		//<! \Structure for kalman filter for left sensor
static kalman_state kalmanTherm;	//<! \Structure for kalman filter for thermal cam !!! USELESS !!!

/*** GLOBAL VARIABLE ***/
SHORT pos;							//<! \Position of the servo motor 0 - 180 °

/*** GLOBAL FUNCTION ***/
BOOL sendFrameTh(const INT *data, BYTE sizeData);
BOOL sendFrameIr(BYTE id, UINT dataIr);
BOOL sendFrameServo(BYTE id, BYTE position);
servo servo_init();
thermal thermal_init();
void servoCommand(servo Servo, INT error);
kalman_state kalman_init(double q, double r, double p, double x);
void kalman_update(kalman_state *state, double measurement);
double normalizeIr(UINT distance);
double *normalizeTherm(INT *therm);
double barycentre(int *matrix);
double getMean(const INT *data, UINT sizeData);
double getSDV(const INT *data, UINT sizeData, double mean);

/*** Prototype function main ***/
static void setup(void);							//!< \Init function of the system
void delay_ms(unsigned int t);						//!< \Wait ms
void init_timer(unsigned int hz);
void idle_task(void *p);
uint8_t tick_interrupt();


/*** WARNING ! MAYBE TURN INTO STATIC SEE RTOS ***/
void taskSensor(void *p);						//!< \Task update of sensor
void taskSerialTxRx(void *p);					//!< \Task serial communication emission
void taskProcessing(void *p);					//!< \Task serial communication reception
void taskTracking(void *p);						//!< \Task tracking


//<! \[+]Setup the TIMER1_COMPA interrupt - it will be our tick interrupt.
TASK_ISR(TIMER1_COMPA_vect, tick_interrupt());


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
	
	cli();			 //<! \Interrupts should remain disabled - they will be enabled as soon as the first task starts executing.
	clr(B,0);
	
	#if UART
	printf("\n~ Board Ready ~\n");
	#endif
	
		
	/*** WAITING ***/
	#if DEBUG
	#else
	_delay_ms(1000);
	#endif
	
	#if LCD
	LCD_command(LCD_CLR); 
	#endif
	
	//<! \[+]The idle task sleeps the CPU  -  set the sleep mode to IDLE,
	// as we need the sleep to be interruptable by the tick interrupt.
	set_sleep_mode(SLEEP_MODE_IDLE);

	initialise_mbox(&mb_dataIrLeft, 0, 0);		//<! \Initialise mailbox to transfer data
	initialise_mbox(&mb_dataIrRight, 0, 0); 
		
	//<! \[+]Create tasks.
	/*** WARNING !!  Priority and Buffer NEED TO BE VERIFY ***/
	create_task(taskSensor, 0, 0, 500U, 100U, 0);		//<! \Size of stack & Priority
	create_task(taskSerialTxRx, 0, 0, 150U, 90U, 0);
	create_task(taskTracking, 0, 0, 1000U, 80U,  0);
	//create_task(taskProcessing, 0, 0, 150U, 80U,  0);

	init_timer(1000U);	//!< \Set TIMER1_COMPA interrupt to tick every 80,000 clock cycles.

	//<! \[+]Start the RTOS - note that this function will never return.
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


/*! \fn		void setup(void)
 *  \brief	Setup the sytem
 */
static void setup(void)
{
	/*** INIT ***/

	port_init();
	
	#if UART
	uart_init(9600);
	#endif
	
	//servo_init();
	adc_init();
	
	#if PWM
	pwm_init();
	#endif
	
	#if LCD
	LCD_init();
	LCD_write("Tracking");
	LCD_command(LCD_LINE2 | 0);		//!< \move cursor  to row 2, position 5
	LCD_write("Team WTF");
	#endif
	
	#if I2C
	if(!twi_init(100000)) //!< \Init I2C  with 100KHz bitrate.
	{
		printf("\nError in initiating I2C interface.");
		while(1);
	}
	#endif
	
	/*** END OF INIT PART ***/
}


/*! \fn		void delay_ms(unsigned int t)
 *  \brief	wait
 *  \param	t : time to wait in ms
 */
void delay_ms(unsigned int t)
{
	wait_for_increment_of(&tick, t, 0, 0);
}


/*! \fn		void taskSensor(void *p)
 *  \brief	Sensor task : acquire, filtering
 *  \param	*p : Pointer to the task
 */
void taskSensor(void *p)
{	
	kalmanIrR = kalman_init(0.5f, 50, 1, 300); //!< \IR sharp Covariance r constant between 40-400mm; Initial mesurement x = 300mm
	kalmanIrL = kalman_init(0.5f, 50, 1, 300);
	
	while(1)	
	{
		//<! \Timing : 1 Sample every 10ms for IR
		if (cpt.cptIr > T_ACQ_IR)
		{
			//distanceIrRight = readInfraredFilter(ADC_CH_IR_RIGHT);	//!< \Moving average filter
			//distanceIRrLeft = readInfraredFilter(ADC_CH_IR_LEFT);
			distanceIrRight = readInfrared(ADC_CH_IR_RIGHT);			//!< \Raw ADC value
			distanceIrLeft = readInfrared(ADC_CH_IR_LEFT);
			
			kalman_update(&kalmanIrR, distanceIrRight);					//!< \Update kalman prediction
			kalman_update(&kalmanIrL, distanceIrLeft);

			distIrRightNorm = normalizeIr((UINT)kalmanIrR.x);			//!< \Normalise value for sensor fusion		
			distIrLeftNorm = normalizeIr((UINT)kalmanIrL.x);
			
			write_mbox_now(&mb_dataIrRight, &kalmanIrR.x);				//!< \Add data to mailboxe
			write_mbox_now(&mb_dataIrLeft, &kalmanIrL.x);
			
			cpt.cptIr = 0;												//<! \Reset cpt
			flagSensorValueChanged = 1;									//!< \Sensor values changed
		}
		
		//<! \Timing : 1 Sample every 40ms for Thermal
		if (cpt.cptTherm > T_ACQ_THERM)
		{
			cpt.cptTherm = 0;																//<! \Reset cpt
						
			thermalDataPtr = mesure_thermal(thermal_Buff, THERMAL_BUFF_SIZE - 1) ;			//<! \Mesure of the thermal
			thermNorm = normalizeTherm(thermalDataPtr);
			
			/* DEBUG */
			//printf("\r\nMeanTherm : %f, SDV : %f", getMean(thermalDataPtr, THERMAL_TP_SIZE), getSDV(thermalDataPtr, THERMAL_TP_SIZE,getMean(thermalDataPtr, THERMAL_TP_SIZE)));
			//printf("\r\nMean : %f", getMean(thermalDataPtr, THERMAL_TP_SIZE));
			//printf("\r\nState : %d", state);
			
			flagSensorValueChanged = 1;					//!< \Sensor values changed
		}
		
		/*** TEST LCD ***/
		//my_itoa(distanceIrRight, lcdBuffer, 10);
		//LCD_write(lcdBuffer);
		//LCD_command(LCD_CLR);
				
		//if((++cpt.cptIr < 255) && (++cpt.cptTherm < 255));	//<! \increment cpt every ms
		cpt.cptIr++;											//<! \increment cpt every ms
		cpt.cptTherm++;											//<! \increment cpt every ms

		uart_putchar('\n');
		uart_putchar('\r');
		uart_putchar('S');
		delay_ms(DELAY_TSENSOR);
	}
}


/*! \fn		void taskSerialTxRx(void *p)
 *  \brief	Task serial communication : receive command, send frame
 *  \param	p* : pointer to the task
 */

volatile char rxData = ' ';		//<! \Variable of receive data

void taskSerialTxRx(void *p)
{
	//static UINT timeout;
	//delay_ms(1000);			//<! \Delay at first

	while(1)
	{	
		///*** TEST IR & KALMAN ***/
		//if(nbrAcqu < 100)
		//{
			//if(flagSensorValueChanged)
			//{
				//printf("\r\n%d ", nbrAcqu);
				//for(i = 0; i < THERMAL_TP_SIZE; i++)
				//{
					//printf("%d ",thermalDataPtr[i]);
				//}
				//
				////printf("\r\n %f \r\n", thermalDataPtr[i+1]);
				//
				////printf("\r\n%d;%d;%f;%d;%f;", i, (INT)kalmanIrL.x, distIrRightNorm, (INT)kalmanIrR.x, distIrLeftNorm);
				//nbrAcqu++;
				//flagSensorValueChanged = 0;
			//}
		//}
		
		if(uart_kbhit())	//<! \If a byte is received
		{		
			if((rxData = uart_getchar()))	//<! \Get the receive char
			{
				if(rxData == CMD_START)		//<! \If the receive char is the header of a command
				{	
					flagRx.start = 1;
					//timeout = cpt.cptTimeoutCpt;
				
					#if VERBOSE
					uart_putchar('\n');
					uart_putchar('C');
					#endif 
				}
				//USART_Flush();
				#if VERBOSE
				uart_putchar(rxData);			//<! \Echo before flush
				#endif
				
				rxData = ' ';
			}
		
			if(flagRx.start)
			{
				//rxData = ' ';
				if ((rxData = uart_getchar()))
				{
					#if VERBOSE
					uart_putchar(rxData);			//<! \Echo before flush
					#endif
					
					/*if((cpt.cptTimeoutCpt - timeout) < TIMEOUT_CMD) break;
						flagRx.start = 0; -> don't rece;
					*/ 
					if (rxData == CMD_IRL)			//<! \If the host require data of IR sensor left
					{
						#if VERBOSE
						uart_putchar('L');
						uart_putchar('\n');
						#endif
					
						/*** Ack ***/
						uart_putchar(CMD_START);
						uart_putchar(CARAC_ACK);
					
						/*** WARNING ! MAYBE INTRODUCE A DELAY HERE ***/
						//_delay_us(DELAY_CMD);
						
						/*** TEST RIR ***/
						sendFrameIr(IR_L_SENSOR, distanceIrLeft);		//<! \Send frame with the Left sensor
					
						flagRx.start = 0;
					}
					else if(rxData == CMD_IRR)	//<! \If the host require data of IR sensor right
					{

						#if VERBOSE
						uart_putchar('R');
						uart_putchar('\n');
						#endif
					
						/*** Ack ***/
						uart_putchar(CMD_START);
						uart_putchar(CARAC_ACK);
							
						/*** WARNING ! MAYBE INTRODUCE A DELAY HERE ***/
						//_delay_us(DELAY_CMD);
										
						/*** TEST RIR ***/
						sendFrameIr(IR_R_SENSOR, distanceIrRight); //<! \Send the frame with data IR right
					
						flagRx.start = 0;
					}
					else if(rxData == CMD_THERM)			//<! \If the host require data of thermal camera
					{
						//printf("\r\nCMD THERM");
						#if VERBOSE
						uart_putchar('T');
						uart_putchar('\n');
						#endif
					
						/*** Ack ***/
						uart_putchar(CMD_START);
						uart_putchar(CARAC_ACK);
										
						/*** WARNING ! MAYBE INTRODUCE A DELAY HERE ***/
						//_delay_us(DELAY_CMD);
										
						/*** TEST THERMAL SENSOR ***/
						sendFrameTh(thermalDataPtr, NBR_DATA);		//<! \Send frame with data from thermal camera
										
						flagRx.start = 0;
					}
					else if(rxData == CMD_SERVO)					//<! \If host require the position of servo
					{
						//printf("\r\nCMD SERVO");
						#if VERBOSE
						uart_putchar('S');
						uart_putchar('\n');
						#endif
					
						/*** Ack ***/
						uart_putchar(CMD_START);
						uart_putchar(CARAC_ACK);
										
						/*** WARNING ! MAYBE INTRODUCE A DELAY HERE ***/
						//_delay_us(DELAY_CMD);
						
						/*** TEST SEND SERVO ***/							
						sendFrameServo(SERVO_MOTOR, pos);					//<! \Send the position of the servo
										
						flagRx.start = 0;
					}
					
					else if(rxData == CMD_SERVO_TURN_LEFT)					//<! \If the host want to move the servo to the left
					{
						rxData = ' ';			
										
						if((rxData = uart_getchar()))
						{
							currentDiffMean = (INT)rxData;
							
							//printf("\r\nCMD SERVO");
							#if VERBOSE
							uart_putchar('L');
							uart_putchar('\n');
							#endif

							/*** Ack ***/
							uart_putchar(CMD_START);
							uart_putchar(CARAC_ACK);
							
							/*** WARNING ! MAYBE INTRODUCE A DELAY HERE ***/
							//_delay_us(DELAY_CMD);
													
							/*** TEST SEND SERVO ***/
							state = STATE_TURN_LEFT;
							//flagTurn = TURN_LEFT;
							flagRx.start = 0;
						}							
					}

					else if(rxData == CMD_SERVO_TURN_RIGHT)			//<! \If the host want to move the servo to the right
					{
						rxData = ' ';
						
						if((rxData = uart_getchar()))
						{
							currentDiffMean = (INT)rxData;
							
							//printf("\r\nCMD SERVO");
							#if VERBOSE
							uart_putchar('R');
							uart_putchar('\n');
							#endif

							/*** Ack ***/
							uart_putchar(CMD_START);
							uart_putchar(CARAC_ACK);
							
						
							/*** WARNING ! MAYBE INTRODUCE A DELAY HERE ***/
						
							/*** TEST SEND SERVO ***/
							state = STATE_TURN_RIGHT;
							//flagTurn = TURN_RIGHT;
							flagRx.start = 0;
						}
					}
					else
					{
						#if VERBOSE
						uart_putchar('U');
						uart_putchar('\n');
						#endif
					
						/*** Non Ack ***/
						uart_putchar(CMD_START);
						uart_putchar(CARAC_NACK);
										
						/*** WARNING ! MAYBE INTRODUCE A DELAY HERE ***/
						
						/*** Go to Iddle state ***/
						state = STATE_IDDLE;
										
						//printf("\r\nCMD UNKNOWN");
						flagRx.start = 0;
					}
					//USART_Flush();
					rxData = ' ';
				}
			}
		}
		
		//uart_putchar('\n');
		//uart_putchar('\r');
		//uart_putchar('T');	
		//if(++cpt.cptTimeoutCpt < 65535);
		//cpt.cptTimeoutCpt++;
		delay_ms(DELAY_TSERIALTX);
	}
}

/*! \fn		void taskTracking(void *p)
 *  \brief	Task tracking : compute centroid from thermal cam and track
 *  \param	*p: pointer to the task
 */

void taskTracking(void *p)
{
	servoT = servo_init();			//<! \Init the servo tracking struct
	thermT = thermal_init();		//<! \Init the thermal struct
	
	servoT.position = servoT.timeMoy;
	
	//pwm_setPosition(servoT.position);
	pwm_setOcr(servoT.position);
	
	servoT.dest = servoT.timeMax;
	servoT.dest_sh = 0;
	
	delay_ms(SERVO_TIME_DEGREE*180);
	
	state = STATE_OBJECT_DETECT;		
	
	while(1)
	{
		switch (state)		//<! \State machine for tracking
		{
		
		case STATE_TURN_LEFT:
			
			if(currentDiffMean != prevDiffMean)
			{
				prevDiffMean = currentDiffMean;
			}
			
			servoT.position -= currentDiffMean/5;
			
			if (servoT.position <= 0)
			{
				servoT.position = 0;
			}
			
			pwm_setPosition(servoT.position);
			
			/*** Return in idle state after turn left ***/
			state = STATE_IDDLE;
			break;
			
		case STATE_TURN_RIGHT:
		
			if(currentDiffMean != prevDiffMean)
			{
				prevDiffMean = currentDiffMean;
			}
			
			servoT.position += currentDiffMean/5;
			
			if (servoT.position >= 180)
			{
				servoT.position = 180;
			}
			
			pwm_setPosition(servoT.position);
			
			/*** Return in idle state after turn right ***/
			state = STATE_IDDLE;
			
			break;
		
		case STATE_IDDLE:

			//servoTrackeur.position = servoTrackeur.position; //<! \For debug useless

			/*** Test implemented function who swith to tracking when person detect ***/
			if(flagSensorValueChanged)
			{
				if(thermNorm[THERMAL_TP_SIZE + 1] > THERM_SEUIL_DETECT)
				{
					printf("\r\nBack to tracking");
					state = STATE_SCAN_LEFT;
				}
				flagSensorValueChanged = 0;
			}
			
			state = STATE_IDDLE;

			break;
			
		case STATE_SCAN_LEFT:	//<! \Turn left to search people
			
			servoT.dest = servoT.timeMin;
			servoT.dest_sh = 0;
			
			while(servoT.dest_sh > servoT.timeMin)
			{
				servoT.dest_sh = (servoT.dest_sh*MINUS_FILTER_SERVO) + servoT.dest*FILTER_SERVO;	//<! \First order filetring for easing servo
				pwm_setOcr(servoT.dest_sh);
				//pwm_setPosition(servoT.dest_sh);
				delay_ms(1);
			}
				
			state = STATE_SCAN_RIGHT;	
			break;
		
		case STATE_SCAN_RIGHT:	//<! \Turn right to search people

			servoT.position = servoT.timeMax;
			servoT.dest_sh = 0;
						
			while(servoT.dest_sh < servoT.timeMax)
			{
				servoT.dest_sh = (servoT.dest_sh*MINUS_FILTER_SERVO) + servoT.dest*FILTER_SERVO;
				pwm_setOcr(servoT.dest_sh);
				//pwm_setPosition(servoT.dest_sh);
				delay_ms(1);
			}
			
			state = STATE_SCAN_LEFT;
			break;
		
		case STATE_OBJECT_DETECT:			//<! \When an object is detect
		
			thermT.bary = 0.0f;
			
			if (flagSensorValueChanged)
			{	
				flagSensorValueChanged = 0;
				
				thermT.bary = barycentre(thermalDataPtr);				//<! \Calcultate centroid of the thermal matrix
				//thermT.translate = thermcentreXT. - thermT.bary;		
				
				//printf("\r\nCog : %f, KL : %d, KR: %d", thermT.bary, (UINT)kalmanIrL.x, (UINT)kalmanIrR.x);
				
				//servoT.position = get_termalTrackingValue(servoT.prevPosition, thermalDataPtr, MILLISECONDS);
				if((thermT.bary > 2.5f) && (((UINT)kalmanIrL.x % (UINT)kalmanIrR.x)) > 50)
				{
					//servoT.position = get_termalTrackingValue(servoT.prevPosition, thermalDataPtr, 2);
			
					//while(servoT.dest_sh < (servoT.position - 100))
					//{
						//servoT.dest_sh = (servoT.dest_sh*MINUS_FILTER_SERVO) + servoT.position*FILTER_SERVO;
						//printf("\r\nDest : %d, DestSh : %d", servoT.position, servoT.dest_sh);
						//pwm_setOcr(servoT.dest_sh);
						////pwm_setPosition(servoT.dest_sh);
						//delay_ms(1);
					//}
					//
					//servoT.prevPosition = servoT.position;
						
					servoT.position += 50*thermT.bary;
					if(servoT.position > servoT.timeMax) servoT.position = servoT.timeMax;
					pwm_setOcr(servoT.position);
					delay_ms(1);
					printf("\r\nPos++ : %d", servoT.position);
				}
				else if((thermT.bary < 2.5f) && (thermT.bary > 0.0f) && (((UINT)kalmanIrR.x % (UINT)kalmanIrL.x)) > 50)
				{
					//servoT.position = get_termalTrackingValue(servoT.prevPosition, thermalDataPtr, 2);
					//
					//while(servoT.dest_sh < (servoT.position - 100))
					//{
					//servoT.dest_sh = (servoT.dest_sh*MINUS_FILTER_SERVO) + servoT.position*FILTER_SERVO;
					//printf("\r\nDest : %d, DestSh : %d", servoT.position, servoT.dest_sh);
					//pwm_setOcr(servoT.dest_sh);
					////pwm_setPosition(servoT.dest_sh);
					//delay_ms(1);
					//}
					//
					//servoT.prevPosition = servoT.position;
					servoT.position -= 50*thermT.bary;
					if(servoT.position < servoT.timeMin) servoT.position = servoT.timeMin;
					pwm_setOcr(servoT.position);
					delay_ms(1);
					printf("\r\nPos-- : %d", servoT.position);
				}
				//else if(((((UINT)kalmanIrL.x % (UINT)kalmanIrR.x)) > 100))	// Object detect a extreme gauche
				//{
					//servoT.position = servoT.timeMin;
					//pwm_setOcr(servoT.position);
					//delay_ms(10);
				//}
				//else if(((((UINT)kalmanIrR.x % (UINT)kalmanIrL.x)) > 100))	// Object detect a extreme droite
				//{
					//servoT.position = servoT.timeMax;
					//pwm_setOcr(servoT.position);
					//delay_ms(10);
				//}
				else servoT.position = servoT.position;
					
				servoT.dest_sh = servoT.dest;
				servoT.dest = 0;
			}

			state = STATE_OBJECT_DETECT;
			break;
			
		default:
		
			state = STATE_IDDLE;
			//<! \Do nothings here

			break;
		}
		
		//uart_putchar('\n');
		//uart_putchar('\r');
		//uart_putchar('T');
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
void taskProcessing(void *p)
{

	while(1)
	{		
		//uart_putchar('\n');
		//uart_putchar('\r');
		//uart_putchar('P');
		delay_ms(DELAY_TPROCESS);	
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
