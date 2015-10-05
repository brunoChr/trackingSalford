/*
 * serialData.h
 *
 * Created: 05/10/2015 11:19:45
 *  Author: b.christol
 */ 


#ifndef SERIALDATA_H_
#define SERIALDATA_H_


/*** INCLUDE ***/

#include "../lib/types.h"


/*** DEFINE DEFINITION ***/

#define IR_R_SENSOR		0x01
#define IR_L_SENSOR		0x02
#define THERMAL_SENSOR	0x03
#define START_BYTE		0x24
#define END_BYTE		0x25


/*** PROTOTYPE ***/

BOOL formatProtocol(BYTE id, BYTE dataLow[8], BYTE dataHigh[8]);


/*** TYPEDEF DEFINITION ***/

typedef struct serialProtocol
{
	BYTE sb;	// Start byte
	BYTE id;	// Identification byte
	BYTE dataLow[8];	// Low part of the data -> MSB first transmission (most significant bit) -> transmission end when transmitting LSB bit
	BYTE dataHigh[8];	// High part of the data
	BYTE cs;			// Cheksum of data, maybe the mean of the 16 byte data ??? or CRC
	BYTE cn;			// data counter, facultative ??
	BYTE eb;			// End byte	
	
} serialProtocol;




#endif /* SERIALDATA_H_ */