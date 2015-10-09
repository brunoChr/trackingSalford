#ifndef THERMAL_H
#define THERMAL_H

#include "generics.h"
#include "twi.h"

#define THERMAL_ADD			0b00001010 // I2C address of thermal camera.
#define THERMAL_BUFF_SIZE	35		   // The byte array frame size.
#define THERMAL_TP_SIZE		16			   //The byte array TP size

//BYTE thermal_data[THERMAL_BUFF_SIZE];

BOOL thermal_read(BYTE address, BYTE *data);
int D6T_checkPEC( BYTE *buf, int pPEC );
BYTE mesure_thermal(BYTE *thermal_Buff, BYTE size);



	
#endif