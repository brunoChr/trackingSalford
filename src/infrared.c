/*
 * \file infrared.c
 *
 * Created: 03/10/2015 11:43:23
 *  Author: b.christol
 */ 

#include "../lib/infrared.h"
#include "../lib/adc.h"


/*** LOCAL FILE VARAIBLE ***/
// Read infrared
static UINT adcResultCh;
//static UINT  distanceIR; // NOT USES AT THE TIME
static UINT sortedValues[NUM_READS_ADC];
static UINT iMed,jMed, kMed;			//<! \i for number af acquisition; j for sorting
static UINT returnval;
static UINT value;						//!< Returned value of the lookup Table

/*** LOCAL FUNCTION PROTOTYPE ***/
static UINT lookupInfrared(UINT adcResul);

/*** GLOBAL FUNCTION PROTOTYPE ***/
UINT readInfraredFilter(BYTE adcPin);
UINT readInfrared(BYTE adcPin);


/*	TODO
	maybe filter : mean, average, khalman Filer
*/	


/*! \fn UINT lookupInfrared(UINT indexLut) 
 *  \brief A member function.
 *  \param c a character.
 *  \return a character pointer.
 */
static UINT lookupInfrared(UINT adcResul)
{	
	/*** HANDLE DATA ***/
	
	adcResul = (adcResul - OFFSET_ADC_LUT);					//!< Applied the offset on the adcResult to match the LUT
	
	if((adcResul >= 0) && (adcResul < SIZE_LUT_IR))			//!< Returned value of the lookup Table if index in range
	{
		value = lookupIR[adcResul];
	}
	else													//!< Handle error  if index in range
	{
		value = 0;
	}
	
	return value;
}



/*** WARNING ! SORTING NOT OPTIMIZED, LOOK LECTURE ON SORTING ***/

/*! \fn UINT readInfrared(BYTE adcPin) 
 *  \brief Acquire IR, sort, apply median and average
 *		   mode filter = median filter + average filter	
 *  \param 
 *  \return Clean IR acquisition
 */
UINT readInfraredFilter(BYTE adcPin)
{
	//<! \read multiple values and sort them to take the mode (median)	
	for(iMed = 0; iMed < NUM_READS_ADC; iMed++)		//<! \Loop for acquisition (10 ~= 400ms) TO VERIFIED
	{		
		adcResultCh = adc_read(adcPin);		//<! \Realize the acquisition								
		
		if((adcResultCh < sortedValues[0]) || (iMed == 0))
		{
			jMed = 0; //<! \insert at first position
		}
		else
		{
			for(jMed = 1; jMed < iMed; jMed++)
			{
				if((sortedValues[jMed-1] <= adcResultCh) && (sortedValues[jMed] >= adcResultCh))
				{
					//<! \j is insert position
					break;
				}
			}
		}
		
		for(kMed = iMed; kMed > jMed; kMed--)
		{
			//<! \move all values higher than current reading up one position
			sortedValues[kMed] = sortedValues[kMed-1];
		}
		
		sortedValues[jMed] = adcResultCh; //<! \insert current reading
	}
	
	
	//<! \return scaled mode of 10 values; TO BE VERIFIED
	
	for(iMed = (NUM_READS_ADC/2) - 5; iMed < ((NUM_READS_ADC/2) + 5); iMed++)
	{
		returnval += sortedValues[iMed];	//<! \Do the sum for average
	}
	
	returnval = (returnval/10);		//<! \Do the division for average, WARNING ! MAYBE USE A DEFINE
	
	return lookupInfrared(returnval);
}


UINT readInfrared(BYTE adcPin)
{
	adcResultCh = adc_read(adcPin);	
	
	return lookupInfrared(adcResultCh);
}

