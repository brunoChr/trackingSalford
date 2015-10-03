#include "../lib/thermal.h"

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


int mesure_thermal(BYTE *thermal_data, BYTE size)
{
	BYTE tPTAT;
	BYTE tP[THERMAL_TP_SIZE];
	BYTE tPEC;
	
	if(!D6T_checkPEC(thermal_data,size)){
		return - 1; // e r r o r
	}
	thermal_read(THERMAL_ADD,thermal_data);
	tPTAT=256*thermal_data[1]+thermal_data[0];
	tP[0]=256*thermal_data[3]+thermal_data[2];
	tP[1]=256*thermal_data[5]+thermal_data[4];
	tP[2]=256*thermal_data[7]+thermal_data[6];
	tP[3]=256*thermal_data[9]+thermal_data[8];
	tP[4]=256*thermal_data[11]+thermal_data[10];
	tP[5]=256*thermal_data[13]+thermal_data[12];
	tP[6]=256*thermal_data[15]+thermal_data[14];
	tP[7]=256*thermal_data[17]+thermal_data[16];
	tP[8]=256*thermal_data[19]+thermal_data[18];
	tP[9]=256*thermal_data[21]+thermal_data[20];
	tP[10]=256*thermal_data[23]+thermal_data[22];
	tP[11]=256*thermal_data[25]+thermal_data[24];
	tP[12]=256*thermal_data[27]+thermal_data[26];
	tP[13]=256*thermal_data[29]+thermal_data[28];
	tP[14]=256*thermal_data[31]+thermal_data[30];
	tP[15]=256*thermal_data[33]+thermal_data[32];
	tPEC=thermal_data[34];
	
	for(int i=0;i<16;i++){
		uart_putchar(tP[i]*128/256);
	}
	return 0;
}