/*
 * servo.h
 *
 * Created: 22/10/2015 14:13:34
 *  Author: b.christol
 */ 


#ifndef SERVO_H_
#define SERVO_H_

#include "../lib/types.h"


#define FRAME_RATE          25    // milliseconds per frame
#define SERVO_SPEED			200   // this the time in ms to move 60 degrees
#define MAX_STEP_PER_FRAME  (60 / (SERVO_SPEED/ FRAME_RATE))	// max degrees servo can move in one frame
#define SERVO_TIME_DEGREE	(SERVO_SPEED/60)					// Time for move of 1°

/*!
 * servo class.
 * \extends
 * \brief
 */ 
typedef struct
{
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
	
	
} servo;


extern servo servo_init();
extern void servoCommand(servo Servo, INT error);

#endif /* SERVO_H_ */