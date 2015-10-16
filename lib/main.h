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
#include <avr/iom128.h>

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

#define TAILLE_DATA 4*4						//!< \Matrix temp sensor
#define ADC_CH_IR_RIGHT	0					//!< \ADC channel of the right IR sensor
#define ADC_CH_IR_LEFT	1					//!< \ADC channel of the left IR sensor
#define DEBUG 0								//!< \Debug macro

#define DELAY_TSENSOR		5					//!< Delay for task sensor in ms
#define DELAY_TSERIALTX		5					//!< Delay for task serial tx in ms
#define DELAY_TSERIALRX		5					//!< Delay for task serial rx in ms
#define DELAY_TTRACKING		50					//!< Delay for task tracking in ms


/*** WARNING MACRO USE FOR DEBUGGING, WILL BE DELETE ***/
#define DEBUG 0
#define LCD	  0
#define UART  1
#define PWM   0
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
};


typedef struct flagReceive
{
	BOOL start;
	
} flagReceive;





#endif /* MAIN_H_ */