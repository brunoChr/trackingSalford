/*
 * \file kalmanFilter.c
 *
 * Created: 05/10/2015 13:28:20
 *  Author: b.christol
 */ 

/*** INCLUDE ***/
#include "../lib/kalmanFilter.h"

/*! \fn		kalman_state kalman_init(double q, double r, double p, double x) 
 *  \brief	Initialize the kalman structure
 *  \param	q: process noise, r: sensor noise, p: , x: initial measurement
 *  \return	Return an initialize kalman struct
 */
kalman_state kalman_init(double q, double r, double p, double x)
{
	kalman_state result;
	 
	result.q= q;	//<! /process noise covariance, A number greater than zero, hight enough 
	result.r= r;	//<! /sensor noise covariance, A number greater than zero
	result.p= p; 
	result.x= x;

	return(result);
}

/*! \fn		void kalman_update(kalman_state *state, double measurement) 
 *  \brief	Update the kalman filter
 *  \param	*state: pointer on a kalman struct, measurement: mesurement to add to the prediction
 */
void kalman_update(kalman_state *state, double measurement)
{
	//<! /[+]Prediction update: 
	state->p = state->p + state->q;
	
	//<! /[+]Measurement update:
	state->k = state->p / (state->p + state->r);
	state->x = state->x + state->k * (measurement - state->x);
	state->p = (1 -state->k) * state->p;
}