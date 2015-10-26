#include "../lib/thermal.h"
#include "../lib/uart.h"

/*** LOCAL FILE VARAIBLE ***/
static INT tPTAT;
static BYTE tP[THERMAL_TP_SIZE];
static INT tPEC;
static int check;
static unsigned char crc;
static int i;
static int indexTherm;
static unsigned char temp;

static BOOL thermal_read(BYTE address, BYTE *data);
static int D6T_checkPEC( BYTE *buf, int pPEC );


/*** TO DO !! 
 *   Resolve error for thermal measurement -> put the temp mesured in INT and not in a BYTE  		
 *   Possible overflow of variable -> false mesurement, when object temp > 25.5°c 
*/


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
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



/*! \fn BYTE * mesure_thermal(const BYTE *thermal_Buff, BYTE size)
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return a character pointer.
 */
BYTE* mesure_thermal(BYTE *thermal_Buff, BYTE size)
{	
	thermal_read(THERMAL_ADD, thermal_Buff);
	
	if(!D6T_checkPEC(thermal_Buff, size))
	{
		return NULL; // e r r o r
	}
		
	tPTAT = (BYTE)(thermal_Buff[1] << 8)|(thermal_Buff[0]);

	tP[0] = (BYTE)(thermal_Buff[3] << 8)|(thermal_Buff[2]);
	tP[1] = (BYTE)(thermal_Buff[5] << 8)|(thermal_Buff[4]);
	tP[2] = (BYTE)(thermal_Buff[7] << 8)|(thermal_Buff[6]);
	tP[3] = (BYTE)(thermal_Buff[9] << 8)|(thermal_Buff[8]);
	tP[4] = (BYTE)(thermal_Buff[11] << 8)|(thermal_Buff[10]);
	tP[5] = (BYTE)(thermal_Buff[13] << 8)|(thermal_Buff[12]);
	tP[6] = (BYTE)(thermal_Buff[15] << 8)|(thermal_Buff[14]);
	tP[7] = (BYTE)(thermal_Buff[17] << 8)|(thermal_Buff[16]);
	tP[8] = (BYTE)(thermal_Buff[19] << 8)|(thermal_Buff[18]);
	tP[9] = (BYTE)(thermal_Buff[21] << 8)|(thermal_Buff[20]);
	tP[10] = (BYTE)(thermal_Buff[23] << 8)|(thermal_Buff[22]);
	tP[11] = (BYTE)(thermal_Buff[25] << 8)|(thermal_Buff[24]);
	tP[12] = (BYTE)(thermal_Buff[27] << 8)|(thermal_Buff[26]);
	tP[13] = (BYTE)(thermal_Buff[29] << 8)|(thermal_Buff[28]);
	tP[14] = (BYTE)(thermal_Buff[31] << 8)|(thermal_Buff[30]);
	tP[15] = (BYTE)(thermal_Buff[33] << 8)|(thermal_Buff[32]);
	
	tPEC = thermal_Buff[34];
	
	//printf("TP : %d", tP);
	//uart_putchar(tPTAT);
	
	//for(int i=0;i<16;i++)
	//{
		//printf("%d ", tP[i]);
	//}
	//
	//printf("\r\n");
	
	
	//uart_putchar(tPEC);
	
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

double gravityCenter(int matrix[])
{
	/*! \fn double gravityCenter(int matrix[])
	*	\brief give back the gravity center of the matrix given in argument
	*	\param matrix : thermal matrix
	*	\exception
	*	\return the gravity center of the termal matrix
	*/
	int i,j = 1;
	double cog_x = 0, sum_x = 0;
	double total_pixelValue = 0;
	
	for(i=0 ; i< THERMAL_TP_SIZE ; i++)
	{
			sum_x += matrix[i] * j;
			j++;
			if ((i%3) == 0 && i>0)
				j = 1;
	
			total_pixelValue += matrix[i];
	}
	
	cog_x = sum_x/total_pixelValue;

	return cog_x;
}


