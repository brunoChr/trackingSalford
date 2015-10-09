#include "../lib/thermal.h"
#include "../lib/uart.h"


	
BOOL thermal_read(BYTE address, BYTE *data)
{
	int check = 0;
	if(twi_start(address, WRITE))
	{
		BYTE check = 0;
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

unsigned char calc_crc( unsigned char data )
{
	int index;
	unsigned char temp;
	for(index=0;index<8;index++){
		temp = data;
		data <<= 1;
		if(temp & 0x80) data ^= 0x07;
		//printf(data);
	}
	return data;
}
int D6T_checkPEC( BYTE *buf, int pPEC )
{
	unsigned char crc;
	int i;
	crc = calc_crc( 0x14 );
	crc = calc_crc( 0x4C ^ crc );
	crc = calc_crc( 0x15 ^ crc );
	for(i=0;i<pPEC;i++){
		crc = calc_crc( buf[i] ^ crc );
	}
	return (crc == buf[pPEC]);
}


BYTE* mesure_thermal(BYTE *thermal_Buff, BYTE size)
{
	static BYTE tPTAT;
	static BYTE tP[THERMAL_TP_SIZE];
	static BYTE tPEC;

	if(!D6T_checkPEC(thermal_Buff, size)){
		return -1; // e r r o r
	}
	
	thermal_read(THERMAL_ADD, thermal_Buff);
	
	tPTAT=256*thermal_Buff[1]+thermal_Buff[0];
	
	tP[0]=256*thermal_Buff[3]+thermal_Buff[2];
	tP[1]=256*thermal_Buff[5]+thermal_Buff[4];
	tP[2]=256*thermal_Buff[7]+thermal_Buff[6];
	tP[3]=256*thermal_Buff[9]+thermal_Buff[8];
	tP[4]=256*thermal_Buff[11]+thermal_Buff[10];
	tP[5]=256*thermal_Buff[13]+thermal_Buff[12];
	tP[6]=256*thermal_Buff[15]+thermal_Buff[14];
	tP[7]=256*thermal_Buff[17]+thermal_Buff[16];
	tP[8]=256*thermal_Buff[19]+thermal_Buff[18];
	tP[9]=256*thermal_Buff[21]+thermal_Buff[20];
	tP[10]=256*thermal_Buff[23]+thermal_Buff[22];
	tP[11]=256*thermal_Buff[25]+thermal_Buff[24];
	tP[12]=256*thermal_Buff[27]+thermal_Buff[26];
	tP[13]=256*thermal_Buff[29]+thermal_Buff[28];
	tP[14]=256*thermal_Buff[31]+thermal_Buff[30];
	tP[15]=256*thermal_Buff[33]+thermal_Buff[32];
	
	tPEC=thermal_Buff[34];
	
	//uart_putchar(tPTAT);
	
	//for(int i=0;i<16;i++)
	//{
		//printf("%d ", tP[i]);
	//}
	//
	//printf("\r\n");
	
	
	//uart_putchar(tPEC);
	
	return thermal_Buff;
}