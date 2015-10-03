/*
 * infrared.c
 *
 * Created: 03/10/2015 11:43:23
 *  Author: b.christol
 */ 


/*** WARNING : VALUES NEED TO BE VERIFIED ***/
/* Sharp GP2Y0A02YK IR Range Sensor - 20 cm to 150 cm */
int sharp_IR_interpret_GP2Y0A02YK(int value)
{
	return 1904.5*pow(value,-.89);
}

