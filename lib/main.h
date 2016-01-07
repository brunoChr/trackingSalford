/*
 * main.h
 *
 * Created: 11/10/2015 22:16:08
 *  Author: b.christol
 */ 


#ifndef MAIN_H_
#define MAIN_H_



#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
//#include <avr/iom128.h>
//#include <avr/iom328.h>
//#include <avr/iom2560.h>

#include "../lib/cpu.h"
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
#include "../lib/rtos.h"
#include "../lib/tracking.h"

#include <util/delay.h>
#include <util/atomic.h>


/*** DEFINE CONST ***/

#define TAILLE_DATA 4*4							//!< \Matrix temp sensor
#define ADC_CH_IR_RIGHT	0						//!< \ADC channel of the right IR sensor
#define ADC_CH_IR_LEFT	1						//!< \ADC channel of the left IR sensor
#define DEBUG 0									//!< \Debug macro

#define DELAY_TSENSOR		1					//!< Delay for task sensor in ms
#define DELAY_TSERIALTX		1					//!< Delay for task serial tx in ms
#define DELAY_TPROCESS		25					//!< Delay for task serial rx in ms
#define DELAY_TTRACKING		25					//!< Delay for task tracking in ms


#define T_ACQ_IR	10	//<! \Acquisition period for IR in ms
#define	T_ACQ_THERM 20	//<! \Acquisition period for thermal in ms
#define TIMEOUT_CMD	5	//<! \Timout in ms for received complete CMD : #x 

#define DELAY_CMD	100	//<! \in us

/*** WARNING MACRO USE FOR DEBUGGING, WILL BE DELETE ***/
#define DEBUG 0
#define LCD	  0
#define UART  1
#define PWM   1
#define I2C	  1	
#define VERBOSE 0		//<! \Make the system chatty


/*** Globalvar ***/


/*** ENUM & STRUCT ***/
/*!
 * enum ReceiveCmd
 * \extends
 * \brief
 */ 
enum ReceiveCmd
{
	CMD_START = '#',
	CMD_IRR = '1',
	CMD_IRL = '2',
	CMD_THERM = '3',
	CMD_SERVO = '4',
	CMD_SERVO_TURN_LEFT = '5',
	CMD_SERVO_TURN_RIGHT = '6',
	CARAC_ACK = 'A',
	CARAC_NACK = 'N',
};


/*!
 * class.
 * \extends 
 * \brief
 */ 
typedef struct flagReceive
{
	BOOL start;
	
} flagReceive;


/*!
 * counter class.
 * \extends cptIr, cptTherm, cptTimeoutCpt
 * \brief Use as a general struct for handle counter
 */ 
typedef struct compteur
{
	BYTE cptIr;
	BYTE cptTherm;
	UINT cptTimeoutCpt;
} compteur;



#endif /* MAIN_H_ */