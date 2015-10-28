/*
 * \file tracking.c
 *
 * Created: 09/10/2015 11:00:00
 *  Author: a.moufounda
 */ 

#include "../lib/pwm.h"
#include "../lib/types.h"
#include "../lib/tracking.h"
#include "../lib/infrared.h"
#include "../lib/thermal.h"
#include <stdio.h>
#include <stdlib.h>

#define OUT_OF_RANGE 0
#define NEED_ROTATION_GAUCHE 1
#define NEED_ROTATION_DROITE 2
#define CIBLE_TRACKEE 3
#define DISTANCE_MAX 1500
#define DISTANCE_MIN		200
#define PAS_DEGREE			(180/400)
#define PAS_TIME_SERVO		0.5f

static 	double position = 0.0f;
static  double cog_x = 0.0f, prevCog_x = 0.0f;
static const double offset = 1000.0f; // 1ms offset : the servo range from 1ms to 2ms
	
	
UINT get_termalTrackingValue(UINT prevPos, int *matrix, int outputType)
{
	/*! \fn UINT get_termalTrackingValue(int matrix[], int outputType);
	*	\brief give back the position for the servomotor, in degrees or duration of the high-state
	*	\param matrix : thermal matrix
	*	\param outputType : the type of position we want (degree or duration)
	*	\exception
	*	\return the new position of the servo, in degrees or duration
	*/
	cog_x = 0.0f;
	position = 1000.0f;
	prevCog_x = 0.0f;
	
	cog_x = barycentre(matrix);
	
	//printf("\r\nBary : %f", cog_x);
	
	if(cog_x != prevCog_x)
	{
		if((cog_x >= 1.0f) && (cog_x <= 4.0f))	position += (cog_x*250.0f);
		else position = prevPos;
		prevCog_x = cog_x;
	}
	else position = prevPos;
	
	
	//if (outputType == DEGREES)
	//{
		//position = (cog_x*100.0f) * PAS_DEGREE;
	//}
	//else if (outputType == MILLISECONDS)
	//{
		//position = ((cog_x*1000.0f) * PAS_TIME_SERVO) + offset;
	//}
	//else
	//{
		////position = (cog_x*1000.0f) * PAS_TIME_SERVO + offset;
	//}
	
	return (UINT)position;
}