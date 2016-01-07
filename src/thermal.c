#include "../lib/thermal.h"
#include "../lib/uart.h"
#include "../lib/fusion.h"
//#include <math.h>

/*** LOCAL FILE VARAIBLE ***/
static INT tPTAT;
static INT tP[THERMAL_TP_SIZE];
static INT tPEC;
static int check;
static unsigned char crc;
//static int i;
static int indexTherm;
static unsigned char temp;
static int i, j = 1;
//static float cog_x = 0;
//static float total_pixelValue = 0;
static double moyenne = 0.0f, ecartType = 0.0f, max = 0.0f;
static double valeurx = 0.0f;
static double nbPoint = 0.0f;
static int cnt = 0;

/*** LOCAL FUNCTION ***/
static BOOL thermal_read(BYTE address, BYTE *data);
static int D6T_checkPEC( BYTE *buf, int pPEC );

/*** GLOBAL FUNCTION ***/
double getMean(const INT *data, UINT sizeData);
double getSDV(const INT *data, UINT sizeData, double mean);


/*** TO DO !! 
 *   Resolve error for thermal measurement -> put the temp mesured in INT and not in a BYTE  		
 *   Possible overflow of variable -> false mesurement, when object temp > 25.5°c 
*/

/*! \fn		thermal thermal_init()
 *  \brief	Init a thermal structure
 *  \return	An initialize thermal structure
 */
thermal thermal_init()
{
	thermal result;
	
	result.centreX = (4/2);
	result.bary = 0.0f;
	
	return(result);
}


/*! \fn		BOOL thermal_read(BYTE address, BYTE *data)
 *  \brief	Read the temperature from thermal camera
 *  \param	address: adress i2c of the sensor
 *  \param	*data: pointer on buffer thermal
 *  \return	error
 */
static BOOL thermal_read(BYTE address, BYTE *data)
{
	if(twi_start(address, WRITE))
	{
		twi_write(0x4C);
		if(twi_start(address, READ))
		{
			twi_read_bytes(data, 35);
			for(BYTE i=0 ; i<34 ; i++)
			check += data[i];
		}
		twi_stop();
	}
	if(check == data[34])
	return(1);
	return(0);
}



/*! \fn		BYTE * mesure_thermal(const BYTE *thermal_Buff, BYTE size)
 *  \brief	Decode the receive frame from the thermal sensor
 *  \param	*thermal_Buff
 *  \param  size: size of the data
 *  \return Pointer on thermal buffer
 */
INT* mesure_thermal(BYTE *thermal_Buff, BYTE size)
{	
	
	thermal_read(THERMAL_ADD, thermal_Buff);
	
	if(!D6T_checkPEC(thermal_Buff, size))
	{
		return NULL; // e r r o r
	}
		
	tPTAT = (thermal_Buff[1] << 8)|(thermal_Buff[0]);

	tP[0] = (thermal_Buff[3] << 8)|(thermal_Buff[2]);
	tP[1] = (thermal_Buff[5] << 8)|(thermal_Buff[4]);
	tP[2] = (thermal_Buff[7] << 8)|(thermal_Buff[6]);
	tP[3] = (thermal_Buff[9] << 8)|(thermal_Buff[8]);
	tP[4] = (thermal_Buff[11] << 8)|(thermal_Buff[10]);
	tP[5] = (thermal_Buff[13] << 8)|(thermal_Buff[12]);
	tP[6] = (thermal_Buff[15] << 8)|(thermal_Buff[14]);
	tP[7] = (thermal_Buff[17] << 8)|(thermal_Buff[16]);
	tP[8] = (thermal_Buff[19] << 8)|(thermal_Buff[18]);
	tP[9] = (thermal_Buff[21] << 8)|(thermal_Buff[20]);
	tP[10] = (thermal_Buff[23] << 8)|(thermal_Buff[22]);
	tP[11] = (thermal_Buff[25] << 8)|(thermal_Buff[24]);
	tP[12] = (thermal_Buff[27] << 8)|(thermal_Buff[26]);
	tP[13] = (thermal_Buff[29] << 8)|(thermal_Buff[28]);
	tP[14] = (thermal_Buff[31] << 8)|(thermal_Buff[30]);
	tP[15] = (thermal_Buff[33] << 8)|(thermal_Buff[32]);
		
	tPEC = thermal_Buff[34];

	return tP;
}

/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
static unsigned char calc_crc( unsigned char data )
{
	for(indexTherm=0;indexTherm<8;indexTherm++){
		temp = data;
		data <<= 1;
		if(temp & 0x80) data ^= 0x07;
		//printf(data);
	}
	return data;
}

/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
static int D6T_checkPEC( BYTE *buf, int pPEC )
{
	crc = calc_crc( 0x14 );
	crc = calc_crc( 0x4C ^ crc );
	crc = calc_crc( 0x15 ^ crc );
	for(i=0;i<pPEC;i++){
		crc = calc_crc( buf[i] ^ crc );
	}
	return (crc == buf[pPEC]);
}

/*! \fn		double barycentre(int *matrix)
 *  \brief	Compute the centroid
 *  \param	*matrix: pointer to the thermal buffer 
 *  \return	The centroid value
 */
double barycentre(int *matrix)
{
	j = 0;
	//sum_x = 0;
	moyenne = 0.0f;
	ecartType = 0.0f;
	//min = 0;
	max = 0.0f;
	valeurx = 0.0f;
	nbPoint = 0.0f;
	cnt = 0;

	moyenne = getMean(matrix, THERMAL_TP_SIZE);					//<! \Compute the mean
	ecartType = getSDV(matrix, THERMAL_TP_SIZE, moyenne);		//<! \Compute the std
	 
	//min = moyenne - ecartType;
	max = moyenne + ecartType;
	
	for (i = 0; i < THERMAL_TP_SIZE; i++)
	{
		j++;
		cnt++;
		//printf("j : %d\n", j);
		//if ((matrix[i] > min) && (matrix[i] < max))
		if (matrix[i] > max)
		{
			valeurx += j;
			nbPoint++;
		}
		if (cnt == 4)
		{
			j = 0;
			cnt = 0;
		}
	}
	
	if(nbPoint != 0.0f)	valeurx /= nbPoint;
	else valeurx = 0.0f;

	return valeurx;
}

