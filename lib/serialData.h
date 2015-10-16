/*!
 *	\file serialData.h
 *	\brief 
 *  Created: 05/10/2015 11:19:45
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
#define SERVO_MOTOR		0x04
#define START_BYTE		0x24
#define END_BYTE		0x25

#define NBR_DATA_THERM	16




/*** TYPEDEF DEFINITION ***/

/*!
 * serialProtocol class.
 * \extends sb, id, dataLow, dataHigh, cs, cn, eb 
 * \brief use following protocol : SB|ID|D0...D15|CS|CN|EB
 */ 
typedef struct serialProtocol
{
	BYTE sb;			/*!< Start byte */ 
	BYTE id;			/*!< Identification byte */
	BYTE data[NBR_DATA_THERM];	/*!< Low part of the data -> MSB first transmission (most significant bit) -> transmission end when transmitting LSB bit */
	//BYTE dataHigh[8];	/*!< High part of the data */
	BYTE cs;			/*!< Cheksum of data, maybe the mean of the 16 byte data ??? or CRC */
	BYTE cn;			/*!< data counter, facultative ?? */
	BYTE eb;			/*!< End byte */
	
} serialProtocol;


/*** VARAIBLE ***/
extern serialProtocol Frame;
extern BYTE indexFrame;

/*** PROTOTYPE ***/

serialProtocol formatProtocol(BYTE id, BYTE *data, INT nbrData);
BYTE checksumCalculation(BYTE *data, BYTE size);
BYTE computeCrc(BYTE *data, BYTE nbrOfBytes);
BOOL sendFrame(BYTE *data, BYTE sizeData);

#endif /* SERIALDATA_H_ */