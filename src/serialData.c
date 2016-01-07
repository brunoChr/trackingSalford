/*!
 * \file serialData.c
 *
 * Created: 05/10/2015 10:34:25
 *  Author: b.christol
 */ 


#include "../lib/serialData.h"
#include <string.h>
#include "../lib/uart.h"
#include "../lib/thermal.h"

/* TODO

*/

/*** LOCAL FILE VARIABLE **/
static serialProtocol trameData;		// Implement a struct data of type serialProtocol -> see serialData.h
static BYTE indexFrame;
	

/*** GLOBAL FUNCTION PROTOTYPE ***/
BOOL sendFrameTh(const INT *data, BYTE sizeData);
BOOL sendFrameIr(BYTE id, UINT dataIr);

/*** LOCAL FUNCTION PROTOTYPE ***/
static BYTE computeCrc(const BYTE *data, BYTE nbrOfBytes);
//static BYTE checksumCalculation(BYTE *data, BYTE size);


/*! \fn		BOOL sendFrameTh(const BYTE *data, BYTE sizeData)
 *  \brief	format data to be send by serial
 *  \param	*data: pointer on thermal data
 *  \param	sizeData: size of the data to send
 *  \return error
 */
BOOL sendFrameTh(const INT *data, BYTE sizeData)
{
	/*** SERIAL PROTOCOL FORMAT ***/
	/*** SB/ID/D0...D15/CS/..CN../EB ***/
		
	trameData.sb = START_BYTE;
	trameData.id = THERMAL_SENSOR;
	
	for (indexFrame = 0; indexFrame < sizeData; indexFrame++)
	{
		trameData.data[indexFrame] = (BYTE)(data[indexFrame] - THERM_OFFSET);
	}
	
	//memcpy(trameData.data, data, sizeData);
	
	trameData.cn = sizeData;
	trameData.cs = computeCrc((BYTE *)data, sizeData);		//<! \Missing data with this cast, TO DO !!
	trameData.eb = END_BYTE;
		
	/*** POSSIBLE ERROR TRAITEMENT ***/
	/*if(data == 0)
	{
		#if VERBOSE
		printf("\nData frame null");
		#endif
		return -1;
	}*/
	
	
	/*** TEST SEND PROTOCOL ***/
	uart_putchar(trameData.sb);
	uart_putchar(trameData.id);
	
	for (indexFrame = 0; indexFrame < NBR_DATA; indexFrame++)
	{
		//printf("%d",Frame.data[indexFrame]);
		uart_putchar(trameData.data[indexFrame]);
	}
	
	uart_putchar(trameData.cs);
	uart_putchar(trameData.cn);
	uart_putchar(trameData.eb);
	
	return 0;
}


/*! \fn		BOOL sendFrameIr(BYTE id, UINT dataIr)
 *  \brief	format data to be send by serial
 *  \param	id: ir sensor 1 or 2
 *  \param	dataIr: distance from sensor
 *  \return error
 */
BOOL sendFrameIr(BYTE id, UINT dataIr)
{
	/*** SERIAL PROTOCOL FORMAT ***/
	/*** SB/ID/D0...D15/CS/..CN../EB ***/
		
	/*** Formatage of the frame ***/
	trameData.sb = START_BYTE;
	trameData.id = id;
	
	trameData.data[0] = (BYTE)(dataIr & 0xFF);					//<! \Send MSByte of distance first
	trameData.data[1] = (BYTE)((dataIr >> 8) & 0xFF);			//<! \Then send the LSbyte of distance
	
	for (indexFrame = 2; indexFrame < NBR_DATA; indexFrame++)
	{
		trameData.data[indexFrame] = 0x00;
	}
			
	trameData.cs = computeCrc(&trameData.data[0], 2);
	trameData.cn = 0x02;
	
	trameData.eb = END_BYTE;
	
	/*** Send the frame ***/	 
	uart_putchar(trameData.sb);
	uart_putchar(trameData.id);
		
	for (indexFrame = 0; indexFrame < NBR_DATA; indexFrame++)
	{
		//printf("%x",trameData.data[indexFrame]);
		uart_putchar(trameData.data[indexFrame]);
	}
		
	uart_putchar(trameData.cs);
	uart_putchar(trameData.cn);
	uart_putchar(trameData.eb);
	
	return 0;
}


/*! \fn		BOOL sendFrameServo(BYTE id, BYTE position)
 *  \brief	format data to be send by serial
 *  \param	position: position to send
 *  \param
 *  \return error
 */
BOOL sendFrameServo(BYTE id, BYTE position)
{
	/*** SERIAL PROTOCOL FORMAT ***/
	/*** SB/ID/D0...D15/CS/..CN../EB ***/
		
	/*** Formatage of the frame ***/
	trameData.sb = START_BYTE;
	trameData.id = id;
	
	trameData.data[0] = position;					//<! \Send Position

	for (indexFrame = 1; indexFrame < NBR_DATA; indexFrame++)
	{
		trameData.data[indexFrame] = 0x00;
	}
			
	trameData.cs = computeCrc(&trameData.data[0], 1);
	trameData.cn = 0x01;
	
	trameData.eb = END_BYTE;
	
	/*** Send the frame ***/	 
	uart_putchar(trameData.sb);
	uart_putchar(trameData.id);
		
	for (indexFrame = 0; indexFrame < NBR_DATA; indexFrame++)
	{
		//printf("%x",trameData.data[indexFrame]);
		uart_putchar(trameData.data[indexFrame]);
	}
		
	uart_putchar(trameData.cs);
	uart_putchar(trameData.cn);
	uart_putchar(trameData.eb);
	
	return 0;
}


/*! \fn		BYTE computeCrc(const BYTE *data, BYTE nbrOfBytes)
 *  \brief	calculates crc for n bytes of data
 *  \param	*data crc is built based on this data
 *  \param	nbrOfBytes crc is built for n bytes of data
 *  \return crc
 */
static BYTE computeCrc(const BYTE *data, BYTE nbrOfBytes)
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
}

