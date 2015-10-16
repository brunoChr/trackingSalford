/*!
 * \file serialData.c
 *
 * Created: 05/10/2015 10:34:25
 *  Author: b.christol
 */ 


#include "../lib/serialData.h"

/* TODO

*/

/*** LOCAL FILE VARAIBLE **/
serialProtocol Frame;
BYTE indexFrame;
	

/*! \fn BOOL formatProtocol(BYTE id, BYTE dataLow[8], BYTE dataHigh[8])
 *  \brief format data to be send by serial
 *  \param 
 *  \param 
 *  \exception 
 *  \return a character pointer.
 */
serialProtocol formatProtocol(BYTE id, BYTE *data, INT nbrData)
{
	/*** SERIAL PROTOCOL FORMAT ***/
	/*** SB/ID/D0...D15/CS/..CN../EB ***/
	
	serialProtocol trameData;		// Implement a struct data of type serialProtocol -> see serialData.h
	
	trameData.sb = START_BYTE;
	trameData.id = id;
	
	memcpy(trameData.data, data, NBR_DATA_THERM);
	//memcpy(trameData.dataHigh, dataHigh, sizeof(BYTE));
	
	trameData.cs = computeCrc(data, NBR_DATA_THERM);
	trameData.eb = END_BYTE;
	
	return trameData;
}


BOOL sendFrame(BYTE *data, BYTE sizeData)
{

	if(data == 0)
	{
		return 1;
	}
	
	/*** TEST FORMAT PROTOCOL ***/
	Frame = formatProtocol(THERMAL_SENSOR, data, sizeData);
	
	uart_putchar(Frame.sb);
	uart_putchar(Frame.id);
	
	for (indexFrame = 0; indexFrame < NBR_DATA_THERM; indexFrame++)
	{
		uart_putchar(Frame.data[indexFrame]);
	}
	
	uart_putchar(Frame.cs);
	uart_putchar(Frame.cn);
	uart_putchar(Frame.eb);
	
	return 0;
}

/*! \fn checksumCalculation(BYTE *data, BYTE size)
 *  \brief compute checksum of data
 *  \param data : pointer on begin of data for calculation of checksum
 *  \param 
 *  \exception 
 *  \return checksum
 */
BYTE checksumCalculation(BYTE *data, BYTE size)
{
	//int array[8];
	int i, num, negative_sum = 0, positive_sum = 0;
	float total = 0.0, average;
	
	num = sizeof(BYTE)*size;
	
	/*  Summation starts */
	for (i = 0; i < num; i++)
	{
		if (data[i] < 0)
		{
			negative_sum = negative_sum + data[i];
		}
		else if (data[i] > 0)
		{
			positive_sum = positive_sum + data[i];
		}
		else if (data[i] == 0)
		{
			;
		}
		total = total + data[i] ;
	}
	
	average = (total / num);
	
	return average;
}


typedef enum
{
	CHECKSUM_ERROR = 0X04
} etError;

//CRC
#define POLYNOMIAL 0x31 //P(x)=x^8+x^5+x^4+1 = 100110001

//============================================================
BYTE computeCrc(BYTE *data, BYTE nbrOfBytes)
//============================================================
//calculates checksum for n bytes of data
//and compares it with expected checksum
//input: data[] checksum is built based on this data
// nbrOfBytes checksum is built for n bytes of data
// checksum expected checksum
//return: error: CHECKSUM_ERROR = checksum does not match
// 0 = checksum matches
//============================================================
{
	BYTE crc = 0;
	BYTE byteCtr;
	BYTE bit;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
	{
		/* Initially, the dividend is the remainder */
		crc ^= (data[byteCtr]);
		
		/*  For each bit position in the message */
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80)/* If the uppermost bit is a 1... */
			crc = (crc << 1) ^ POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	
	return crc;
	//if (crc != checksum)
	//return CHECKSUM_ERROR;
	//else return 0;
}


/*** Example of protocol

|SYNC|DEST|SOURCE|DATA1|DATA0|CHKSUM|
the checksum is generated like
temp = SYNC + DEST + SOURCE + DATA1 + DATA0;
CHKSUM = 0 - TEMP;
now when you receive the data only have to check
if the sum of all the recieved data is 0.

***/