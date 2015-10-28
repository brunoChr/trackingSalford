/*
 * fusion.c
 *
 * Created: 27/10/2015 11:09:15
 *  Author: b.christol
 */ 

#include "../lib/fusion.h"
#include "../lib/infrared.h"
#include "../lib/thermal.h"
#include <math.h>

/*** LOCAL FILE VARIABLE ***/
static UINT sumMean = 0, indexLoop = 0;
static double sum_deviation = 0.0f;

/*** LOCAL FUNCTION PROTOTYPE ***/

/*** GLOBAL FUNCTION PROTOTYPE ***/
double normalizeIr(UINT distance);
double *normalizeTherm(INT *therm);
double getMean(const INT *data, UINT sizeData);
double getSDV(const INT *data, UINT sizeData, double mean);

/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
double getMean(const INT *data, UINT sizeData)
{
	sumMean = 0;
	
	for(indexLoop = sizeData; indexLoop != 0; --indexLoop)
	{
		sumMean += data[indexLoop];
	} 
	
	return (double)(sumMean/sizeData);
}


double getSDV(const INT *data, UINT sizeData, double mean)
{
	sum_deviation = 0.0f;
	
	for(indexLoop = sizeData; indexLoop != 0; --indexLoop)
		sum_deviation += (data[indexLoop]-mean)*(data[indexLoop]-mean);
	
	return sqrt(sum_deviation/sizeData);
}


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
	static double thermNorm[THERMAL_TP_SIZE];
	//static double sumTherm;
	/*
	Centered: X ? mean									-> centering just makes the mean of the data equal to 0.
	Standardized: (X ? mean)/sd							-> Standardizing turns the data into z-scores
	Normalized:  (X ? min(X)) / (max(X) ? min(X))		-> Normalizing in this sense rescales the data to the unit interval.
	*/
	//sumTherm = 0;
	
	for (int i = 0; i < THERMAL_TP_SIZE; i++)
	{	
		//sumTherm += therm[i];
		thermNorm[i] = ( 1- ((therm[i] - THERM_MIN)/(THERM_MAX - THERM_MIN)));
		
	}
	
	//thermNorm[THERMAL_TP_SIZE+1] = (sumTherm / THERMAL_TP_SIZE);
	
	return thermNorm;
}

