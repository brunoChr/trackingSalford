/*
 * fusion.h
 *
 * Created: 27/10/2015 11:09:26
 *  Author: b.christol
 */ 


#ifndef FUSION_H_
#define FUSION_H_

/*** INCLUDE ***/
#include "../lib/types.h"

/*** DEFINE ***/


/*** GLOBAL VARIABLE ***/


/*** PROTOTYPE GLOBAL FUNCTION ***/
extern double normalizeIr(UINT distance);
extern double *normalizeTherm(INT *therm);
extern double getMean(const INT *data, UINT sizeData);
extern double getSDV(const INT *data, UINT sizeData, double mean);

#endif /* FUSION_H_ */