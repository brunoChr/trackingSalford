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
