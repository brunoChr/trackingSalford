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
	
	double q; // Process noise covariance.
	double r; // sensor noise, Measurement noise covariance.
	double x; // filtered value, Estimate value.
	double p; // Estimate error covariance.
	double k; // Kalman gain.

} kalman_state;


/*** Global function ***/
extern kalman_state kalman_init(double q, double r, double p, double x);
extern void kalman_update(kalman_state *state, double measurement);

#endif /* KALMANFILTER_H_ */