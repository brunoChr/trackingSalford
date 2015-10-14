/*
 * \file infrared.c
 *
 * Created: 03/10/2015 11:43:23
 *  Author: b.christol
 */ 

#include "../lib/infrared.h"
#include "../lib/adc.h"

/* SENSOR SHARP GP2Y0A02YK CARACTERISTIC */
/* • Detection Accuracy @ 80 cm: ±10 cm	*/
/* • Range: 20 to 150 cm */
/* • Typical response time: 39 ms */
/* • Typical start up delay: 44 ms */
/* • Average Current Consumption: 33 mA */


/*	TODO
	maybe filter : mean, average, khalman Filer
	
	average = total / samples.
	new_average = (total + new) / (samples + 1)
	
	Example code of average 
	
	//add val, return avg
	//assuming 8bit samples
	//avg only valid after 150 samples

	uint8_t add_sample(uint8_t val){
		static uint16_t total; //sum of 150 samples
		static uint8_t index; //buf index
		static uint8_t buf[150]; //store 150 samples

		if(index>149){ //if overflow
			index=0; //reset index
		}
		total -= buf[count]; //sub 'earliest' value
		total += val; //add new value
		buf[count++]=val; //store new value

		return total / 150; //return average
	}
*/


/*! \fn UINT lookupInfrared(UINT indexLut) 
 *  \brief A member function.
 *  \param c a character.
 *  \return a character pointer.
 */

UINT lookupInfrared(UINT adcResul)
{
	/*** LOCAL VARIABLE ***/
	UINT value = 0;						//!< Returned value of the lookup Table
	
	/*!
	*	Const LUT for infrared
	*/
	static const __flash UINT lookupIR[SIZE_LUT_IR] =
	{
		1507U,1495U,1483U,1472U,1460U,1448U,1437U,1426U,1415U,1403U,1392U,
		1381U,1371U,1360U,1349U,1339U,1328U,1318U,1307U,1297U,1287U,1277U,
		1267U,1257U,1247U,1238U,1228U,1219U,1209U,1200U,1191U,1181U,1172U,
		1163U,1154U,1146U,1137U,1128U,1120U,1111U,1103U,1094U,1086U,1078U,
		1070U,1061U,1054U,1046U,1038U,1030U,1022U,1015U,1007U,1000U,992U,
		985U,978U,970U,963U,956U,949U,942U,936U,929U,922U,915U,909U,902U,
		896U,889U,883U,877U,871U,865U,858U,852U,847U,841U,835U,829U,823U,
		818U,812U,807U,801U,796U,790U,785U,780U,774U,769U,764U,759U,754U,
		749U,744U,740U,735U,730U,725U,721U,716U,712U,707U,703U,698U,694U,
		690U,686U,681U,677U,673U,669U,665U,661U,657U,653U,649U,646U,642U,
		638U,635U,631U,627U,624U,620U,617U,613U,610U,607U,603U,600U,597U,
		594U,591U,587U,584U,581U,578U,575U,572U,570U,567U,564U,561U,558U,
		556U,553U,550U,547U,545U,542U,540U,537U,535U,532U,530U,527U,525U,
		523U,520U,518U,516U,514U,512U,509U,507U,505U,503U,501U,499U,497U,
		495U,493U,491U,489U,487U,485U,483U,481U,480U,478U,476U,474U,473U,
		471U,469U,468U,466U,464U,463U,461U,460U,458U,456U,455U,453U,452U,
		450U,449U,448U,446U,445U,443U,442U,441U,439U,438U,437U,435U,434U,
		433U,432U,430U,429U,428U,427U,426U,424U,423U,422U,421U,420U,419U,
		418U,416U,415U,414U,413U,412U,411U,410U,409U,408U,407U,406U,405U,
		404U,403U,402U,401U,400U,399U,398U,397U,396U,396U,395U,394U,393U,
		392U,391U,390U,389U,388U,388U,387U,386U,385U,384U,383U,382U,382U,
		381U,380U,379U,378U,377U,377U,376U,375U,374U,373U,373U,372U,371U,
		370U,369U,369U,368U,367U,366U,365U,365U,364U,363U,362U,361U,361U,
		360U,359U,358U,358U,357U,356U,355U,354U,354U,353U,352U,351U,351U,
		350U,349U,348U,347U,347U,346U,345U,344U,343U,343U,342U,341U,340U,
		339U,339U,338U,337U,336U,335U,334U,334U,333U,332U,331U,330U,329U,
		329U,328U,327U,326U,325U,324U,324U,323U,322U,321U,320U,319U,318U,
		317U,317U,316U,315U,314U,313U,312U,311U,310U,309U,309U,308U,307U,
		306U,305U,304U,303U,302U,301U,300U,299U,298U,297U,296U,295U,294U,
		294U,293U,292U,291U,290U,289U,288U,287U,286U,285U,284U,283U,282U,
		281U,280U,279U,278U,277U,276U,275U,274U,273U,272U,271U,270U,269U,
		268U,266U,265U,264U,263U,262U,261U,260U,259U,258U,257U,256U,255U,
		254U,253U,252U,251U,250U,249U,248U,247U,246U,244U,243U,242U,241U,
		240U,239U,238U,237U,236U,235U,234U,233U,232U,231U,230U,229U,228U,
		226U,225U,224U,223U,222U,221U,220U,219U,218U,217U,216U,215U,214U,
		213U,212U,211U,210U,209U,208U,207U,206U,205U,204U,203U,202U,201U,
		200U,199U,198U,197U,196U,196U,195U,194U,193U,192U,191U,190U,189U,
		188U,188U,187U,186U,185U,184U,183U,183U,182U,181U,180U,179U,179U,
		178U,177U,176U,176U,175U,174U,174U,173U,172U,172U,171U,170U,170U,
		169U,169U,168U,167U,167U,166U,166U,165U,165U,164U,164U,163U,163U,
		163U,162U
	};
	
	
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

/*! \fn UINT lookupInfrared(UINT indexLut) 
 *  \brief Acquire IR, sort, apply median and average
 *  \param 
 *  \return Clean IR acquisition
 */
UINT readInfrared(BYTE adcPin)
{
	UINT adcResultCh;
	UINT  distanceIR;
	
	//<! \read multiple values and sort them to take the mode (median)
	UINT sortedValues[NUM_READS];
	UINT i;
	
	for(i=0; i < NUM_READS; i++)
	{		
		adcResultCh = adc_read(adcPin);
		int j;
		
		if((adcResultCh < sortedValues[0]) || (i == 0))
		{
			j = 0; //<! \insert at first position
		}
		else
		{
			for(j = 1; j < i; j++)
			{
				if((sortedValues[j-1] <= adcResultCh) && (sortedValues[j] >= adcResultCh))
				{
					//<! \j is insert position
					break;
				}
			}
		}
		for(int k = i; k > j; k--)
		{
			//<! \move all values higher than current reading up one position
			sortedValues[k] = sortedValues[k-1];
		}
		
		sortedValues[j] = adcResultCh; //<! \insert current reading
	}
	
	//<! \return scaled mode of 10 values
	UINT returnval = 0;
	
	for(int i = (NUM_READS/2) - 5; i < ((NUM_READS/2) + 5); i++)
	{
		returnval += sortedValues[i];
	}
	
	returnval = (returnval/10);
	
	return lookupInfrared(returnval);
}




