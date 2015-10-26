#ifndef THERMAL_H
#define THERMAL_H

/*** INCLUDE ***/
#include "generics.h"
#include "twi.h"

/*** DEFINE DEFINITION ***/
#define THERMAL_ADD			0b00001010 // I2C address of thermal camera.
#define THERMAL_BUFF_SIZE	35		   // The byte array frame size.
#define THERMAL_TP_SIZE		16		   //The byte array TP size
#define THERM_OFFSET		100		   //  Applied a offset on therm data to fit in a BYTE, range is now between 10�c and 35.5�C

//#define NB_LINE				4			//line number of the thermal matrix
//#define NB_ROW				4			//row number of the thermal matrix
#define DEGREES				1			//Indication for the position type we want for the PWM
#define MILLISECONDS		2			//Indication for the type of position we want for the PWM

///*** GLOBALS STATIC VARIABLE ***/
//extern BYTE tPTAT;
//extern BYTE tP[THERMAL_TP_SIZE];
//extern BYTE tPEC;
//extern int check;	
//extern unsigned char crc;
//extern int i;
//extern int indexTherm;
//extern unsigned char temp;
//
///*** GLOBAL PROTOTYPE FUNCTION ***/
//extern BOOL thermal_read(BYTE address, BYTE *data);
//extern int D6T_checkPEC( BYTE *buf, int pPEC );
INT * mesure_thermal(BYTE *thermal_Buff, BYTE size);
double gravityCenter(int matrix[]);



	
#endif