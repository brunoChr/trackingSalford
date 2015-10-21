/*
 * kalmanFilter.h
 *
 * Created: 05/10/2015 13:28:28
 *  Author: b.christol
 */ 


#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_


/*** INCLUDE ***/
#include "../lib/types.h"


typedef struct { 
	
	double q; // Process noise covariance. double r; // Measurement noise covariance.
	double r; // Measurement noise covariance.
	double x; // Estimate value.

	double p; // Estimate error covariance.

	double k; // Kalmangain.

} kalman_state;




#endif /* KALMANFILTER_H_ */