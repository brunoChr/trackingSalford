/*
 * \file kalmanFilter.c
 *
 * Created: 05/10/2015 13:28:20
 *  Author: b.christol
 */ 

/*** INCLUDE ***/
#include "../lib/kalmanFilter.h"


kalman_state kalman_init(double q, double r, double p, double x)
{
	kalman_state result;
	 
	result.q= q; 
	result.r= r;
	result.p= p; 
	result.x= x;

	return(result);
}

void kalman_update(kalman_state *state, double measurement)
{

	// [+]Prediction update: 
	// omit x_t= x_t-1 state->p = state->p + state->q;
	state->p = state->p + state->q;

	// [+]Measurement update:
	state->k = state->p / (state->p + state->r);
	state->x = state->x + state->k * (measurement -state->x);
	state->p = (1 -state->k) * state->p;
}