#ifndef TRACKING_H
#define TRACKING_H


#define DEGREES				1			//<! \Indication for the position type we want for the PWM
#define MILLISECONDS		2			//<! \Indication for the type of position we want for the PWM


UINT tracking(int position, const UINT *ptrDistL,const UINT *ptrDistR);
UINT get_termalTrackingValue(UINT prevPos, int *matrix, int outputType);

#endif