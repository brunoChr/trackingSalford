/**
 * Created by Manfred Langemann, Peter Fleury, Theo Theodoridis.
 * File     : twi.c
 * Version  : v1.0
 * Date     : © Copyright 04-01-2008
 * User     : ttheod
 * email    : ttheod@gmail.com
 * Comments : None.
 **/

#include "cpu.h"
#include <stdio.h>
#include <avr/interrupt.h>

#include "generics.h"
#include "twi.h"

// [+]General TWI Master status codes:
#define TWI_START					0x08  // START has been transmitted.
#define TWI_REP_START				0x10  // Repeated START has been transmitted.
#define TWI_ARB_LOST				0x38  // Arbitration lost.

// [+]TWI Master Transmitter status codes:
#define TWI_MTX_ADR_ACK				0x18  // SLA+W has been transmitted and ACK received.
#define TWI_MTX_ADR_NACK			0x20  // SLA+W has been transmitted and NACK received. 
#define TWI_MTX_DATA_ACK			0x28  // Data byte has been transmitted and ACK received.
#define TWI_MTX_DATA_NACK			0x30  // Data byte has been transmitted and NACK received. 

// [+]TWI Master Receiver status codes:
#define TWI_MRX_ADR_ACK				0x40  // SLA+R has been transmitted and ACK received.
#define TWI_MRX_ADR_NACK			0x48  // SLA+R has been transmitted and NACK received.
#define TWI_MRX_DATA_ACK			0x50  // Data byte has been received and ACK transmitted.
#define TWI_MRX_DATA_NACK			0x58  // Data byte has been received and NACK transmitted.

// [+]TWI Slave Transmitter status codes:
#define TWI_STX_ADR_ACK				0xA8  // Own SLA+R has been received; ACK has been returned.
#define TWI_STX_ADR_ACK_M_ARB_LOST	0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned.
#define TWI_STX_DATA_ACK			0xB8  // Data byte in TWDR has been transmitted; ACK has been received.
#define TWI_STX_DATA_NACK			0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received.
#define TWI_STX_DATA_ACK_LAST_BYTE	0xC8  // Last data byte in TWDR has been transmitted (TWEA = "0"); ACK has been received.

// [+]TWI Slave Receiver status codes:
#define TWI_SRX_ADR_ACK				0x60  // Own SLA+W has been received ACK has been returned.
#define TWI_SRX_ADR_ACK_M_ARB_LOST	0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned.
#define TWI_SRX_GEN_ACK				0x70  // General call address has been received; ACK has been returned.
#define TWI_SRX_GEN_ACK_M_ARB_LOST	0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned.
#define TWI_SRX_ADR_DATA_ACK		0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned.
#define TWI_SRX_ADR_DATA_NACK		0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned.
#define TWI_SRX_GEN_DATA_ACK		0x90  // Previously addressed with general call; data has been received; ACK has been returned.
#define TWI_SRX_GEN_DATA_NACK		0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned.
#define TWI_SRX_STOP_RESTART		0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave.

// [+]TWI Miscellaneous status codes:
#define TWI_NO_STATE				0xF8  // No relevant state information available; TWINT = "0".
#define TWI_BUS_ERROR				0x00  // Bus error due to an illegal START or STOP condition.

/**
 * Function   : twi::twi_init()
 * Purpose    : To initialize the TWI master interface.
 * Parameters : - bit_rate : The TWI bitrate frequency (Hz).
 * Returns    : - FALSE    :	 Bitrate too high.
 *				- TRUE     : Bitrate OK.
 * Notes      : None.
 **/
BOOL twi_init(LONG bit_rate)
{
	// [+]Set TWI bitrate.
	// If bitrate is too high, then error return:
	TWBR = ((F_CPU / bit_rate) - 16) / 2;
	if(TWBR < 11)
	return(FALSE);
	return(TRUE);
}

/**
 * Function   : twi::twi_start()
 * Purpose    : To start the TWI master interface.
 * Parameters : - address  : The device address.
 *				- twi_type : The type of required Operation:
 *							 TWIM_READ  - Read data from the slave.
 *							 TWIM_WRITE - Write data to the slave.
 * Returns    : - TRUE     : OK, TWI Master accessible.
 *				- FALSE    : Error in starting TWI Master.
 * Notes      : None.
 **/
BOOL twi_start(BYTE address, BYTE twi_type)
{
	BYTE twst;
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);	// Send START condition.
	while(!(TWCR & (1<<TWINT)));				// Wait until transmission completed.

	// [+]Check value of TWI Status Register. Mask prescaler bits:
	twst = TWSR & 0xF8;
	if((twst != TWI_START) && (twst != TWI_REP_START)) return(FALSE);

	// [+]Send device address:
	TWDR = (address<<1) + twi_type;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// [+]Wait until transmission completed and ACK/NACK has been received:
	while(!(TWCR & (1<<TWINT)));

	// [+]Check value of TWI Status Register. Mask prescaler bits:
	twst = TWSR & 0xF8;
	if((twst != TWI_MTX_ADR_ACK) && (twst != TWI_MRX_ADR_ACK))
	return(FALSE);
	return(TRUE);
}

/**
 * Function   : twi::twi_stop()
 * Purpose    : To stop the TWI master.
 * Parameters : None
 * Returns    : Nothing.
 * Notes      : None.
 **/
void twi_stop(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); // Send stop condition.
	while(TWCR & (1<<TWINT)); // Wait until stop condition is executed and bus released.
}

/**
 * Function   : twi::twi_write()
 * Purpose    : To write a byte to the slave.
 * Parameters : - byte  : The byte to send.
 * Returns    : - TRUE  : OK, Byte sent
 *				- FALSE : Error in byte transmission.
 * Notes      : None.
 **/
BOOL twi_write(BYTE byte)
{
	BYTE twst;

	// [+]Send data to the previously addressed device:
	TWDR = byte;
	TWCR = (1<<TWINT)|(1<<TWEN);

	// [+]Wait until transmission completed:
	while(!(TWCR & (1<<TWINT)));

	// [+]Check value of TWI Status Register. Mask prescaler bits:
	twst = TWSR & 0xF8;
	if(twst != TWI_MTX_DATA_ACK)
	return(TRUE);
	return(FALSE);
}

/**
 * Function   : twi::twi_read_ack()
 * Purpose    : To read a byte from the slave and request next byte.
 * Parameters : None.
 * Returns    : The last byte.
 * Notes      : None.
 **/
BYTE twi_read_ack(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	return(TWDR);
}

/**
 * Function   : twi::twi_read_nak()
 * Purpose    : To read the last byte from a slave.
 * Parameters : None.
 * Returns    : The last byte.
 * Notes      : None.
 **/
BYTE twi_read_nak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	return(TWDR);
}

/**
 * Function   : twi::twi_write_dev()
 * Purpose    : To write on a device.
 * Parameters : - address : The device's address.
 *				- reg     : The device's register.
 *				- data    : The byte data to write.
 * Returns    : Nothing.
 * Notes      : Check device's datasheet for address and register.
 **/
void twi_write_dev(BYTE address, BYTE reg, BYTE data)
{
	TWCR = 0xA4;			// Send a start bit on i2c bus.
	while(!(TWCR & 0x80));	// Wait for confirmation of transmit.
	TWDR = address;			// Load address of i2c device.
	TWCR = 0x84;			// Transmit.
	while(!(TWCR & 0x80));	// Wait for confirmation of transmit.
	TWDR = reg;
	TWCR = 0x84;			// Transmit.
	while(!(TWCR & 0x80));	// Wait for confirmation of transmit.
	TWDR = data;
	TWCR = 0x84;			// Transmit.
	while(!(TWCR & 0x80));	// Wait for confirmation of transmit.
	TWCR = 0x94;			// Stop bit.
}

/**
 * Function   : twi::twi_read_dev()
 * Purpose    : To read a device.
 * Parameters : - address : The device's address.
 *				- reg     : The device's register.
 * Returns    : The value read from the device.
 * Notes      : Check device's datasheet for address and register.
 **/
BYTE twi_read_dev(BYTE address, BYTE reg)
{
	BYTE read_data = 0;

	TWCR = 0xA4;			// Send a start bit on i2c bus.
	while(!(TWCR & 0x80));	// Wait for confirmation of transmit.
	TWDR = address;			// Load address of i2c device.
	TWCR = 0x84;			// Transmit.
	while(!(TWCR & 0x80));	// Wait for confirmation of transmit.
	TWDR = reg;				// Send register number to read from.
	TWCR = 0x84;			// Transmit.
	while(!(TWCR & 0x80));	// Wait for confirmation of transmit.

	TWCR = 0xA4;			// Send repeated start bit.
	while(!(TWCR & 0x80));	// Wait for confirmation of transmit.
	TWDR = address+1;		// Transmit address of i2c device with read bit set.
	TWCR = 0xC4;			// Clear transmit interrupt flag.
	while(!(TWCR & 0x80));	// Wait for confirmation of transmit.
	TWCR = 0x84;			// Transmit, nack (last byte request).
	while(!(TWCR & 0x80));	// Wait for confirmation of transmit
	read_data = TWDR;		// and grab the target data.
	TWCR = 0x94;			// Send a stop bit on i2c bus.
	return(read_data);
}

/**
 * Function   : twi::twi_read_bytes()
 * Purpose    : To receive a byte array.
 * Parameters : - data : The data array.
 *				- size : The size of the array.
 * Returns    : Nothing.
 * Notes      : None.
 **/
void twi_read_bytes(BYTE *data, BYTE size)
{
	for(BYTE i=0 ; i<size ; i++)
	{
		if(i < (size - 1)) data[i] = twi_read_ack();
		else			   data[i] = twi_read_nak();
	}
}

