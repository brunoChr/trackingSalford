/*
 * servo.c
 *
 * Created: 22/10/2015 14:13:24
 *  Author: b.christol
 */ 

#include "../lib/servo.h"
#include "../lib/pwm.h"

/*! \fn		servo servo_init()
 *  \brief	Init a servo structure
 *  \param	none
 *  \return An initialize servo structure
 */
servo servo_init()
{
	servo result;
	
	result.timeMin = SERVO_TIME_LOW;
	result.timeMax = SERVO_TIME_HIGH;
	result.timeMoy = ((result.timeMax + result.timeMin)/2);
	result.posMax = 180;
	result.posMin = 0;
	result.posCenter = ((result.posMax - result.posMin) / 2);
	result.position = result.posCenter;
	result.error = 0;
	result.prevError = 0x8000U;
	result.dgain = 500;
	result.igain = 1;
	result.pgain = 300;
	result.prevPosition = 0;
	
	return(result);
}

/*! \fn		void servoCommand(servo Servo, INT error)
 *  \brief	PID for servo : MISSED FEEDBACK VALUE, can be encoder, or reconstruct speed from EMF
 *  \param	Servo: strucure servo, error: error between command and feedback
 */
void servoCommand(servo Servo, INT error)
{	
	double speed;
		
	if(Servo.prevError != 0x8000U)
	{
		speed = (error*Servo.pgain + (error - Servo.prevError)*Servo.dgain);

		Servo.position += (INT)speed;
		
		if(Servo.position > Servo.posMax) Servo.position =  Servo.posMax;	
		else if(Servo.position < Servo.posMin) Servo.position =  Servo.posMin;		
	}

	pwm_setPosition(Servo.position);
			
	Servo.prevError = error;
	Servo.prevPosition = Servo.position;
	
}

/*! \fn		float servoEaseOutQuad (float t, float b, float c, float d)
 *  \brief	quadratic easing out - decelerating to zero velocity
 *  \param	t: current time, b: beginning value, c: change in value, d: duration
 *  \return The easing value
 */
float servoEaseOutQuad (float t, float b, float c, float d)
{
	return -c *(t/=d)*(t-2) + b;
}
