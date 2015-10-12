/*
 * \file tracking.c
 *
 * Created: 09/10/2015 11:00:00
 *  Author: a.moufounda
 */ 

#include "../lib/pwm.h"
#include <stdio.h>
#include <stdlib.h>

#define OUT_OF_RANGE 0
#define NEED_ROTATION_GAUCHE 1
#define NEED_ROTATION_DROITE 2
#define CIBLE_TRACKEE 3
#define DISTANCE_MAX 80
#define DISTANCE_MIN 20

/*! \fn int info_tracking(int8_t distanceIrRight, int8_t distanceIrLeft)
 *  \brief inform about the direction the pwm need to follow
 *  \param distanceIrRight : distance read by the right infrared sensor
 *  \param distanceIrLeft : distance read by the left infrared sensor
 *  \exception target focused ; target out of range
 *  \return direction
 */

int info_tracking(int8_t distanceIrRight, int8_t distanceIrLeft)
{
	//!< if the value from both right and left sensors are out of the range (distance > 150 cm)
	if ((distanceIrRight >= DISTANCE_MAX) && (distanceIrLeft >= DISTANCE_MAX))
	{
		return OUT_OF_RANGE;		
	}
	//!< if the value from both right and left sensors are out of the range (distance < 20 cm)
	else if((distanceIrRight < DISTANCE_MIN) || (distanceIrLeft < DISTANCE_MIN))
	{
		return OUT_OF_RANGE;
	}
	//!< if someone is in front of the left sensor but not in front of the right one, the indication must be to turn on left
	else if((distanceIrRight >= DISTANCE_MAX) && (distanceIrLeft < DISTANCE_MAX))
	{
		return NEED_ROTATION_GAUCHE;
	}
	//!< if someone is in front of the right sensor but not in front of the left one, the indication must be to turn on right
	else if((distanceIrRight < DISTANCE_MAX) && (distanceIrLeft >= DISTANCE_MAX))
	{
		return NEED_ROTATION_DROITE;
	}
	//!< if someone is in front of the both sensors, the target is well-focused
	else if((distanceIrRight < DISTANCE_MAX) && (distanceIrLeft < DISTANCE_MAX))
	{
		return CIBLE_TRACKEE;
	}
	
	return 0;
}

/*! \fn void tracking(int8_t distanceIrRight, int8_t distanceIrLeft)
 *  \brief follow the target according to the information given by the function "info_tracking"
 *  \param distanceIrRight : distance read by the right infrared sensor
 *  \param distanceIrLeft : distance read by the left infrared sensor
 *  \exception 
 *  \return 
 */
void tracking(int8_t distanceIrRight, int8_t distanceIrLeft)
{
	int i = 0;
	while ((info_tracking(distanceIrRight, distanceIrLeft) != CIBLE_TRACKEE)
			&& (info_tracking(distanceIrRight, distanceIrLeft) != OUT_OF_RANGE))
	{
		if(info_tracking(distanceIrRight, distanceIrLeft) == NEED_ROTATION_DROITE)
		{
			if(i < 0)
			//!< if the pwm previously went to the left, 'i' must be reinitialized
			{
				i = 0;
			}
			pwm_setPosition(91 + i);
			i++;
		}
		
		else if(info_tracking(distanceIrRight, distanceIrLeft) == NEED_ROTATION_GAUCHE)
		{
			//!< if the pwm previously went to the right, 'i' must be reinitialized
			if(i > 0)
			{
				i = 0;
			}
			pwm_setPosition(89 + i);
			i--;
		}
	}
}