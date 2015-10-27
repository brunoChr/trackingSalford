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


/*** LOCAL VARIABLE, MOVE TO .h ***/
static BYTE thermal_Buff[THERMAL_BUFF_SIZE];					//!< \Buffer of temp, static because only use in main.c
static INT *thermalDataPtr;									//!< \Pointer to the buffer temp
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
static INT angleServo;
static mailbox_t SerialData;
static volatile INT currentDiffMean;
static BYTE prevDiffMean;

static servo servoT;

static kalman_state kalmanIrR;
static kalman_state kalmanIrL;
static kalman_state kalmanTherm;

static BOOL flagDiffReceive;
static int dur = 100; //duration is 100 loops
				

/*** GLOBAL VARIABLE ***/
SHORT pos;


/*** GLOBAL FUNCTION ***/
BOOL sendFrameTh(const INT *data, BYTE sizeData);
BOOL sendFrameIr(BYTE id, UINT dataIr);
BOOL sendFrameServo(BYTE id, BYTE position);
servo servo_init();
void servoCommand(servo Servo, INT error);
kalman_state kalman_init(double q, double r, double p, double x);
void kalman_update(kalman_state *state, double measurement);
double normalizeIr(UINT distance);
double *normalizeTherm(INT *therm);
float barycentre(int *matrix);

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


// [+]Setup the TIMER1_COMPA interrupt - it will be our tick interrupt.
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
	
	cli();			 // Interrupts should remain disabled - they will be enabled as soon as the first task starts executing.
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
	

	// [+]The idle task sleeps the CPU  -  set the sleep mode to IDLE,
	// as we need the sleep to be interruptable by the tick interrupt.
	set_sleep_mode(SLEEP_MODE_IDLE);

	#if UART
	//printf("\n%d;%d", distanceIrRight, distanceIRrLeft);
	#endif

	// [+]Create tasks.
	/*** WARNING !!  Priority and Buffer NEED TO BE VERIFY ***/
	create_task(taskSensor, 0, 0, 150U, 100U, 0);		//<! \Size of stack & Priority
	create_task(taskSerialTxRx, 0, 0, 150U, 90U, 0);
	create_task(taskTracking, 0, 0, 100U, 80U,  0);
	//create_task(taskProcessing, 0, 0, 150U, 80U,  0);
	

	init_timer(1000U);	//!< \Set TIMER1_COMPA interrupt to tick every 80,000 clock cycles.

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
	LCD_command(LCD_LINE2 | 0); // move cursor  to row 2, position 5
	LCD_write("Team WTF");
	#endif
	
	#if I2C
	if(!twi_init(100000)) // Init I2C  with 100KHz bitrate.
	{
		printf("\nError in initiating I2C interface.");
		while(1);
	}
	#endif
	
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
	kalmanIrR = kalman_init(1, 100, 1, 300); // IR sharp Covariance r constant between 40-400mm; Initial mesurement x = 300mm
	kalmanIrL = kalman_init(1, 100, 1, 300);
		
	//int i = 0;
	//delay_ms(1000);
	
	while(1)	
	{
		//printf("\nUart : %d", uart_kbhit());
		//uart_putchar('a');

		//<! \Timing : 1 Sample every 10ms for IR
		if (cpt.cptIr == T_ACQ_IR)
		{
			//distanceIrRight = readInfraredFilter(ADC_CH_IR_RIGHT);	// Mod filter
			//distanceIRrLeft = readInfraredFilter(ADC_CH_IR_LEFT);
			distanceIrRight = readInfrared(ADC_CH_IR_RIGHT);			// Raw value
			distanceIrLeft = readInfrared(ADC_CH_IR_LEFT);
			
			kalman_update(&kalmanIrR, distanceIrRight);
			kalman_update(&kalmanIrL, distanceIrLeft);

			distIrRightNorm = normalizeIr((UINT)kalmanIrR.x);			
			distIrLeftNorm = normalizeIr((UINT)kalmanIrL.x);
			
			cpt.cptIr = 0;								//<! \Reset cpt
			//flagSensorValueChanged = 1;
		}
		
		//<! \Timing : 1 Sample every 40ms for Thermal
		if (cpt.cptTherm == T_ACQ_THERM)
		{
			thermalDataPtr = mesure_thermal(thermal_Buff, THERMAL_BUFF_SIZE - 1) ;			//<! \Mesure of the thermal
			thermNorm = normalizeTherm(thermalDataPtr);
			
			if(thermNorm[THERMAL_TP_SIZE + 1] > THERM_SEUIL_DETECT) state = STATE_OBJECT_DETECT;
			
			//printf("\r\n%d", get_termalTrackingValue(thermalDataPtr, DEGREES));
			cpt.cptTherm = 0;																//<! \Reset cpt
			flagSensorValueChanged = 1;
		}
		
		
		/*** TEST LCD ***/
		//my_itoa(distanceIrRight, lcdBuffer, 10);
		//LCD_write(lcdBuffer);
		//LCD_command(LCD_CLR);
				
		//if((++cpt.cptIr < 255) && (++cpt.cptTherm < 255)); //<! \increment cpt every ms
		cpt.cptIr++;
		cpt.cptTherm++;

		uart_putchar('\n');
		uart_putchar('\r');
		uart_putchar('S');
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
	//static UINT timeout;
	//delay_ms(1000);
	
	static int i = 0;

	while(1)
	{	
		/*** TEST IR & KALMAN ***/
		//if(i < 100)
		//{
			//if(flagSensorValueChanged)
			//{
				//for(i = 0; i < THERMAL_TP_SIZE; i++)
				//{
					//printf("%f ",thermNorm[i]);
				//}
				//
				//printf("\r\n %f \r\n", thermNorm[i+1]);
				//
				////printf("\r\n%d;%d;%f;%d;%f;", i, (INT)kalmanIrL.x, distIrRightNorm, (INT)kalmanIrR.x, distIrLeftNorm);
				//i++;
				//flagSensorValueChanged = 0;
			//}
		//}
		//
		//if(uart_kbhit())
		//{
			//uart_flush();
			//i = 0;
		//}
		//printf("\n\rKnbit : %d", uart_kbhit());
		
		//if(uart_kbhit())
		if(0)
		{		
			if((rxData = uart_getchar()))
			{
				if(rxData == CMD_START)
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
					if (rxData == CMD_IRL)
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
						sendFrameIr(IR_L_SENSOR, distanceIrLeft);
					
						flagRx.start = 0;
					}
					else if(rxData == CMD_IRR)
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
						sendFrameIr(IR_R_SENSOR, distanceIrRight);
					
						flagRx.start = 0;
					}
					else if(rxData == CMD_THERM)
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
						sendFrameTh(thermalDataPtr, NBR_DATA);
										
						flagRx.start = 0;
					}
					else if(rxData == CMD_SERVO)
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
							
						sendFrameServo(SERVO_MOTOR, pos);
										
						/*** TEST SEND SERVO ***/
						flagRx.start = 0;
					}
					
					else if(rxData == CMD_SERVO_TURN_LEFT)
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

					else if(rxData == CMD_SERVO_TURN_RIGHT)
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



/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */

void taskTracking(void *p)
{
	servoT = servo_init();
	
	state = STATE_OBJECT_DETECT;

	servoT.position = servoT.timeMoy;
	
	//pwm_setPosition(servoT.position);
	pwm_setOcr(servoT.position);
	
	servoT.dest = servoT.timeMax;
	servoT.dest_sh = 0;
	
	delay_ms(SERVO_TIME_DEGREE*180);
	
	
	//pos = 90;
	//int newPos = 0;
	
	while(1)
	{
		switch (state)
		{
		case STATE_TURN_LEFT:
			
			//servoTrackeur.position -= (currentDiffMean/10);
			
			if(currentDiffMean != prevDiffMean)
			{
				prevDiffMean = currentDiffMean;
			}
			
			//servoCommand(servoTrackeur, (prevDiffMean - currentDiffMean));
			
			servoT.position -= currentDiffMean/5;
			
			//pwm_setPosition(servoTrackeur.position);
			
			//servoTrackeur.position = servoTrackeur.position - 10;
			
			if (servoT.position <= 0)
			{
				servoT.position = 0;
			}
			
			pwm_setPosition(servoT.position);
			
			state = STATE_IDDLE;
			
			break;
			
		case STATE_TURN_RIGHT:
		
			//servoTrackeur.position += (currentDiffMean/10);
			//servoTrackeur.error = (currentDiffMean - prevDiffMean);
			//servoCommand(servoTrackeur);
			//prevDiffMean = currentDiffMean;
					
			//servoTrackeur.position = servoTrackeur.position + 10;

			if(currentDiffMean != prevDiffMean)
			{
				prevDiffMean = currentDiffMean;
			}
			
			servoT.position += currentDiffMean/5;
			
			//servoCommand(servoTrackeur, (prevDiffMean - currentDiffMean));
			
			if (servoT.position >= 180)
			{
				servoT.position = 180;
			}
			
			pwm_setPosition(servoT.position);
			
			state = STATE_IDDLE;
			
			break;
		
		case STATE_IDDLE:

			//servoTrackeur.position = servoTrackeur.position; //<! \Useless
			//state = STATE_IDDLE;
			
			//printf("\r\n %f \r\n", thermNorm[THERMAL_TP_SIZE + 1]);
			
			//if(flagSensorValueChanged)
			//{
				//if(thermNorm[THERMAL_TP_SIZE + 1] > THERM_SEUIL_DETECT)
				//{
					//printf("\r\nBack to tracking");
					//state = STATE_SCAN_LEFT;
				//}
				//flagSensorValueChanged = 0;
			//}
			
			break;
			
		case STATE_SCAN_LEFT:
		
			for(servoT.position = servoT.timeMin; servoT.position < servoT.timeMax; servoT.position += 5)
			{
				pwm_setOcr(servoT.position);
				//pwm_setPosition(servoT.position);
				//servoTrackeur.position -= 1;
				delay_ms(SERVO_TIME_DEGREE);
			}
			
			//servoT.dest = servoT.timeMin;
			//servoT.dest_sh = 0;
			//
			//while(servoT.dest_sh > servoT.timeMin)
			//{
				//servoT.dest_sh = (servoT.dest_sh*MINUS_FILTER_SERVO) + servoT.dest*FILTER_SERVO;
				//pwm_setOcr(servoT.dest_sh);
				////pwm_setPosition(servoT.dest_sh);
				//delay_ms(1);
			//}
			//
			state = STATE_SCAN_RIGHT;
			
			//servoTrackeur.error = 180 - pos;
			//servoCommand(servoTrackeur);
			
			//servoTrackeur.position -= 10;
			//if ((servoTrackeur.position -= 10) < 0)
			//{
				//servoTrackeur.position = 0;
				//printf("\r\nAngle zero");
				//state = STATE_SCAN_RIGHT;
			//}
			//else
			//{
				//state = STATE_SCAN_LEFT;
			//}
				
			break;
		
		case STATE_SCAN_RIGHT:

			for(servoT.position = servoT.timeMax; servoT.position > servoT.timeMin; servoT.position -= 5)
			{
				pwm_setOcr(servoT.position);
				//pwm_setPosition(servoT.position);
				//servoTrackeur.position -= 1;
				delay_ms(SERVO_TIME_DEGREE);
			}
			
			//servoT.dest = servoT.timeMax;
			//servoT.dest_sh = 0;
						//
			//while(servoT.dest_sh < servoT.timeMax)
			//{
				//servoT.dest_sh = (servoT.dest_sh*MINUS_FILTER_SERVO) + servoT.dest*FILTER_SERVO;
				//pwm_setOcr(servoT.dest_sh);
				////pwm_setPosition(servoT.dest_sh);
				//delay_ms(1);
			//}
						
			state = STATE_SCAN_LEFT;
			
			//servoTrackeur.position += 10;
			//if ((servoTrackeur.position += 10) > 180)
			//{
				//servoTrackeur.position = 180;
				//printf("\r\nAngle 180");
				//state = STATE_SCAN_LEFT;
			//}
			//else
			//{
				//state = STATE_SCAN_RIGHT;
			//}
			
			break;
		
		case STATE_OBJECT_DETECT:
		
			pwm_setOcr(servoT.timeMoy);
			
			if(flagSensorValueChanged)
			{
				printf("\r\n Barycentre : %f", barycentre(thermalDataPtr));
				//
				//if(thermNorm[THERMAL_TP_SIZE + 1] > THERM_SEUIL_DETECT)
				//{
					//printf("\r\nBack to tracking");
					//state = STATE_SCAN_LEFT;
				//}
				flagSensorValueChanged = 0;
			}
					
			//printf("\r\n %f \r\n", thermNorm[THERMAL_TP_SIZE + 1]);
			
			//while(servoT.dest_sh < servoT.timeMax) 
			//{
			//servoT.dest_sh = (servoT.dest_sh*MINUS_FILTER_SERVO) + servoT.dest*FILTER_SERVO;	
			//pwm_setOcr(servoT.dest_sh);
			////pwm_setPosition(servoT.dest_sh);
			//delay_ms(1);
			//}
			
			 //for (int pos=0; pos<500; pos++){
				 ////move servo from 0 and 140 degrees forward
				 //OCR3A = (int)servoEaseOutQuad(pos, 1000, 2000, dur);
				 //delay_ms(50); //wait for the servo to move
			 //}
		
			//if(servoT.dest_sh > 179) state = STATE_IDDLE;
			//state = STATE_IDDLE;
			
			break;
			
		default:
		
			//<! \Do nothings here
			
			//servoTrackeur.position = 0;
			
			break;
		}
		
		
		//pwm_setPosition(servoT.dest_sh);
		//printf("\r\nAngle : %u", servoTrackeur.position);
		
		//delay_ms(50);
		
		///*** TEST PWM SERVO ***/
		/*pwm_rotationDroite();
		_delay_ms(500);
		pwm_positionCentrale();
		_delay_ms(500);
		pwm_rotationGauche();
		_delay_ms(500);*/
		
		uart_putchar('\n');
		uart_putchar('\r');
		uart_putchar('T');
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
