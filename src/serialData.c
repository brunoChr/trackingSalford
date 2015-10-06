/*
 * serialData.c
 *
 * Created: 05/10/2015 10:34:25
 *  Author: b.christol
 */ 


#include "../lib/serialData.h"

/* TODO

*/

BOOL formatProtocol(BYTE id, BYTE dataLow[8], BYTE dataHigh[8])
{
	/*** SERIAL PROTOCOL FORMAT ***/
	/*** SB/ID/D0...D15/CS/..CN../EB ***/
		
	serialProtocol trameData;		// Implement a struct data of type serialProtocol -> see serialData.h
	
	trameData.sb = START_BYTE;
	trameData.id = id;
	
	memcpy(trameData.dataLow, dataLow, sizeof(BYTE));
	memcpy(trameData.dataHigh, dataHigh, sizeof(BYTE));
	
	//trameData.cs = mean(dataLow+dataHigh);
	trameData.eb = END_BYTE;
	
	return 0;
}


void checksumCalculation(BYTE *data)
{
	int array[8];
	int i, num, negative_sum = 0, positive_sum = 0;
	float total = 0.0, average;
	
	num = 16;
	
	/*  Summation starts */
	for (i = 0; i < num; i++)
	{
		if (array[i] < 0)
		{
			negative_sum = negative_sum + array[i];
		}
		else if (array[i] > 0)
		{
			positive_sum = positive_sum + array[i];
		}
		else if (array[i] == 0)
		{
			;
		}
		total = total + array[i] ;
	}
	
	average = total / num;
	
}


/*** Example of protocol

|SYNC|DEST|SOURCE|DATA1|DATA0|CHKSUM|
the checksum is generated like
temp = SYNC + DEST + SOURCE + DATA1 + DATA0;
CHKSUM = 0 - TEMP;
now when you receive the data only have to check
if the sum of all the recieved data is 0.

***/