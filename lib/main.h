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

#include <util/delay.h>
#include <util/atomic.h>


/*** DEFINE CONST ***/

#define TAILLE_DATA 4*4						//!< \Matrix temp sensor
#define ADC_CH_IR_RIGHT	0					//!< \ADC channel of the right IR sensor
#define ADC_CH_IR_LEFT	1					//!< \ADC channel of the left IR sensor
#define DEBUG 0								//!< \Debug macro


/*** Globalvar ***/

BYTE thermal_Buff[THERMAL_BUFF_SIZE];		//!< \Buffer of temp
BYTE *thermalDataPtr;						//!< \Pointer to the buffer temp
static semaphore_t tick = {0};				//!< \A semaphore is incremented at every tick.


/*** Prototype function main ***/

BYTE * getRandom();							//!< \Return a 16 byte array fill with random number
void setup(void);							//!< \Init function of the system
void delay_ms(unsigned int t);				//!< \Wait ms
void taskTracking(void *p);					//!< \Task tracking
void taskSerialRx(void *p);					//!< \Task serial communication reception
void taskSerialTx(void *p);					//!< \Task serial communication emission
void taskSensor(void *p);					//!< \Task update of sensor
void init_timer(unsigned int hz);
void idle_task(void *p);
uint8_t tick_interrupt();

#endif /* MAIN_H_ */