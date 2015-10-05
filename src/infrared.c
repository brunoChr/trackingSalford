/*
 * infrared.c
 *
 * Created: 03/10/2015 11:43:23
 *  Author: b.christol
 */ 

#include "../lib/infrared.h"

/*	TODO
	maybe filter : mean, average, khalman Filer
	
	average = total / samples.
	new_average = (total + new) / (samples + 1)
	
	Example code of average 
	
	//add val, return avg
	//assuming 8bit samples
	//avg only valid after 150 samples

	uint8_t add_sample(uint8_t val){
		static uint16_t total; //sum of 150 samples
		static uint8_t index; //buf index
		static uint8_t buf[150]; //store 150 samples

		if(index>149){ //if overflow
			index=0; //reset index
		}
		total -= buf[count]; //sub 'earliest' value
		total += val; //add new value
		buf[count++]=val; //store new value

		return total / 150; //return average
	}
*/


/*** WARNING : VALUES NEED TO BE VERIFIED ***/
/* Sharp GP2Y0A02YK IR Range Sensor - 20 cm to 150 cm */
int sharp_IR_interpret_GP2Y0A02YK(int value)
{
	return 1904.5*pow(value,-.89);
}


/*** mV -> centimeters based on log regression

float gp2y0a02ykGetCentimerDistanceForTension(float milliVolt) {
	// This value are based on Excel logarithm regression!
	return -47.82f * log(milliVolt) + 352.11f;
}
***/

