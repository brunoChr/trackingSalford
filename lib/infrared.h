/*!
* \file infrared.h
*
*  Created: 05/10/2015 15:26:31
*  Author: b.christol
*/


#ifndef INFRARED_H_
#define INFRARED_H_


/*** INCLUDE ***/

#include "../lib/types.h"

/*** DEFINE ***/

#define SIZE_LUT_IR		512U		//!< Size of the look Up table, 512 int : 1024 byte in rom memory
#define OFFSET_ADC_LUT	68U			//!< Offset to applied to Adc result to match the look up table
#define NUM_READS_ADC	10U


/*** GLOBAL STATIC VARIABLE ***/

// Read infrared
extern UINT adcResultCh;
//extern  distanceIR; // NOT USES AT THE TIME
extern UINT sortedValues[NUM_READS_ADC];
extern UINT iMed,jMed, kMed;	//<! \i for number af acquisition; j for sorting
extern UINT returnval;
// LUT VARIABLE
extern UINT value;						//!< Returned value of the lookup Table
	
	
/*** PROTOTYPE FUNCTION ***/

//int sharp_IR_interpret_GP2Y0A02YK(int value);
//extern UINT lookupInfrared(UINT indexLut);
extern UINT readInfrared(BYTE adcPin);

#endif /* INFRARED_H_ */