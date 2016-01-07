/*
 * servo.h
 *
 * Created: 22/10/2015 14:13:34
 *  Author: b.christol
 */ 


#ifndef SERVO_H_
#define SERVO_H_

#include "../lib/types.h"


/* SERVO TOWER PRO SG-5010 CARACTERISTIC *
 • Stall Torque: 20kg/cm @ 6.0V
 • Speed: 0.2 seconds/60deg. @ 4.8V
 • Torque:	4.8V: 76.4 oz-in (5.50 kg-cm)
			6.0V: 90.3 oz-in (6.50 kg-cm)
 • Speed:	4.8V: 0.19 sec/60°
			6.0V: 0.15 sec/60°
 • Dimensions: 66x30.2x64.4mm
 • Temperature Range: 0c - 55c
 • Operating Voltage: 4.8V - 7.2V
*/


#define SERVO_TIME_LOW	1000  //<! \in us
#define SERVO_TIME_HIGH 2000
#define SERVO_TIME_MIDDLE ((SERVO_TIME_HIGH + SERVO_TIME_LOW)/2)  

#define FRAME_RATE          25    //<! \milliseconds per frame
#define SERVO_SPEED			200   //<! \this the time in ms to move 60 degrees
#define MAX_STEP_PER_FRAME  (60 / (SERVO_SPEED/ FRAME_RATE))	//<! \max degrees servo can move in one frame
#define SERVO_TIME_DEGREE	(SERVO_SPEED/60)					//<! \Time for move of 1°


/*** EASING ***/
#define  FILTER_SERVO			0.05f
#define  MINUS_FILTER_SERVO		(1.0f - FILTER_SERVO)

/*!
 * servo class.
 * \extends
 * \brief
 */ 
typedef struct
{
	INT timeMin;
	INT timeMax;
	INT timeMoy;
	INT position;
	INT prevPosition;
	INT posMax;
	INT posMin;
	INT posCenter;
	INT error;
	INT prevError;
	INT pgain; 	 //<! \.
	INT igain; 	 //<! \.
	INT dgain; 	 //<! \.
	INT dest;
	INT dest_sh;

} servo;



extern servo servo_init();
extern void servoCommand(servo Servo, INT error);

#endif /* SERVO_H_ */