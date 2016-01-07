#ifndef THERMAL_H
#define THERMAL_H

/*** INCLUDE ***/
#include "generics.h"
#include "twi.h"

/*** DEFINE DEFINITION ***/
#define THERMAL_ADD			0b00001010	//<! \I2C address of thermal camera.
#define THERMAL_BUFF_SIZE	35			//<! \The byte array frame size.
#define THERMAL_TP_SIZE		16			//<! \The byte array TP size
#define THERM_OFFSET		100			//<! \Applied a offset on therm data to fit in a BYTE, range is now between 10°c and 35.5°C

//#define NB_LINE				4			//<! \line number of the thermal matrix
//#define NB_ROW				4			//<! \row number of the thermal matrix

#define THERM_MIN	0.0f
#define THERM_MAX	400.0f
#define THERM_SEUIL_DETECT 270.0f


/*!
 * servo class.
 * \extends
 * \brief
 */ 
typedef struct
{
	INT centreX;
	double bary;
	INT translate;

} thermal;


///*** GLOBALS STATIC VARIABLE ***/

///*** GLOBAL PROTOTYPE FUNCTION ***/
INT * mesure_thermal(BYTE *thermal_Buff, BYTE size);
double gravityCenter(int *matrix);
extern double barycentre(int *matrix);
extern thermal thermal_init();

	
#endif