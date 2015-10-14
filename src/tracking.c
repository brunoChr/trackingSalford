/*
 * \file tracking.c
 *
 * Created: 09/10/2015 11:00:00
 *  Author: a.moufounda
 */ 

#include "../lib/pwm.h"
#include "../lib/types.h"
#include "../lib/tracking.h"
#include <stdio.h>
#include <stdlib.h>

#define OUT_OF_RANGE 0
#define NEED_ROTATION_GAUCHE 1
#define NEED_ROTATION_DROITE 2
#define CIBLE_TRACKEE 3
#define DISTANCE_MAX 1500
#define DISTANCE_MIN 200

/*! \fn int info_tracking(int8_t distanceIrRight, int8_t distanceIrLeft)
 *  \brief inform about the direction the pwm need to follow
 *  \param distanceIrRight : distance read by the right infrared sensor
 *  \param distanceIrLeft : distance read by the left infrared sensor
 *  \exception target focused ; target out of range
 *  \return direction
 */

int info_tracking(UINT distanceIrRight, UINT distanceIrLeft)
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
	else if(((distanceIrRight >= DISTANCE_MAX) || (distanceIrRight < DISTANCE_MIN)) &&
	 ((distanceIrLeft < DISTANCE_MAX) && (distanceIrLeft >= DISTANCE_MIN)))
	{
		return NEED_ROTATION_GAUCHE;
	}
	//!< if someone is in front of the right sensor but not in front of the left one, the indication must be to turn on right
	else if(((distanceIrLeft>= DISTANCE_MAX) || (distanceIrLeft < DISTANCE_MIN)) &&
	((distanceIrRight < DISTANCE_MAX) && (distanceIrRight >= DISTANCE_MIN)))
	{
		return NEED_ROTATION_DROITE;
	}
	//!< if someone is in front of the both sensors, the target is well-focused
	else if(((distanceIrRight < DISTANCE_MAX) && (distanceIrRight >= DISTANCE_MIN))
	 && ((distanceIrLeft < DISTANCE_MAX) && (distanceIrLeft >= DISTANCE_MIN)))
	{
		return CIBLE_TRACKEE;
	}
	
	return 0;
}

unsigned int tracking(int position)
{
	/*! \fn unsigned int tracking(UINT distanceIrRight, UINT distanceIrLeft, int position)
	*	\brief follow the target according to the information given by the function "info_tracking"
	*	\param distanceIrRight : distance read by the right infrared sensor
	*	\param distanceIrLeft : distance read by the left infrared sensor
	*	\param position : actual position of the servomotor, in degrees
	*	\exception
	*	\return the new position of the servo, in degrees
	*/
	UINT distanceIrRight, distanceIrLeft;
	
	distanceIrRight = readInfrared(0);
	distanceIRrLeft = readInfrared(1);
	
	switch(info_tracking(distanceIrRight, distanceIrLeft))
	{
		case OUT_OF_RANGE:
			pwm_positionCentrale();
			return 90;
			break;
		
		case NEED_ROTATION_DROITE:
			position++;
			pwm_setPosition(position);
			return position;
			break;
			
		case NEED_ROTATION_GAUCHE:
			position--;
			pwm_setPosition(position);
			return position;
			break;
		
		case CIBLE_TRACKEE:
			pwm_setPosition(position);
			return position;
			
		default:
			return position;
			break;
	}
	return position;				
}