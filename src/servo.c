/*
 * servo.c
 *
 * Created: 22/10/2015 14:13:24
 *  Author: b.christol
 */ 

#include "../lib/servo.h"
#include "../lib/pwm.h"

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


// quadratic easing out - decelerating to zero velocity
// t: current time, b: beginning value, c: change in value, d: duration
float servoEaseOutQuad (float t, float b, float c, float d)
{
	return -c *(t/=d)*(t-2) + b;
}

/* Example of calling this function 
 
 int dur = 100; //duration is 100 loops
 for (int pos=0; pos<dur; pos++){
	 //move servo from 0 and 140 degrees forward
	 myServo.write(Easing::easeInOutCubic(pos, 0, 140, dur));
	 delay(15); //wait for the servo to move
 }
*/

//void smoothmove(int cposition, int oldposition) {
	//int moveAngle;
//
	//while(cposition != oldposition) {
//
		//moveAngle = oldposition + MAX_STEP_PER_FRAME; //this is the raw uneased value
		//if( moveAngle > cposition)
		//moveAngle = cposition;
//
		//// code needed using similar logic for cposition < oldposition
//
		//oldposition += Rad2Angle( 0.5 - 0.5 * cos(Angle2Rad(moveAngle) * PI) );
		//myservo.write(oldposition);
		//myservos.refresh();
	//}
//}