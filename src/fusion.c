/*
 * fusion.c
 *
 * Created: 27/10/2015 11:09:15
 *  Author: b.christol
 */ 

#include "../lib/fusion.h"
#include "../lib/infrared.h"
#include "../lib/thermal.h"

/*** LOCAL FILE VARIABLE ***/

/*** LOCAL FUNCTION PROTOTYPE ***/

/*** GLOBAL FUNCTION PROTOTYPE ***/
double normalizeIr(UINT distance);
double *normalizeTherm(INT *therm);

/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
double normalizeIr(UINT distance)
{
	/*
	Centered: X ? mean									-> centering just makes the mean of the data equal to 0.
	Standardized: (X ? mean)/sd							-> Standardizing turns the data into z-scores
	Normalized:  (X ? min(X)) / (max(X) ? min(X))		-> Normalizing in this sense rescales the data to the unit interval.
	*/
	
	return (1- ((distance-IR_MIN)/(IR_MAX-IR_MIN)));
}


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
double *normalizeTherm(INT *therm)
{
	static double thermNorm[THERMAL_TP_SIZE + 1];
	static double sumTherm;
	/*
	Centered: X ? mean									-> centering just makes the mean of the data equal to 0.
	Standardized: (X ? mean)/sd							-> Standardizing turns the data into z-scores
	Normalized:  (X ? min(X)) / (max(X) ? min(X))		-> Normalizing in this sense rescales the data to the unit interval.
	*/
	sumTherm = 0;
	
	for (int i = 0; i < THERMAL_TP_SIZE; i++)
	{	
		sumTherm += therm[i];
		thermNorm[i] = ( 1- ((therm[i] - THERM_MIN)/(THERM_MAX - THERM_MIN)));
		
	}
	
	thermNorm[THERMAL_TP_SIZE+1] = (sumTherm / THERMAL_TP_SIZE);
	
	return thermNorm;
}

