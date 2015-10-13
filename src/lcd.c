/*
 * lcd.c
 *
 * Created: 13/10/2015 16:33:22
 *  Author: b.christol
 */ 

/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
/*
VERY IMPORTANT
For ATmega1281/2561 change device in Project > Configuration options
For ATmega128 unset ATmega103 Compatibilty fuse (run AVRISP, Fuses tab, uncheck M103C fuse
and do Device > Program Fuses)
*/

//Program to drive LCD on STK300 board
// normally for Hitachi type 16 x 2 LCD, but will work on other LCD sizes
//This program uses Busy Flag on LCD (PORTA.7) instead of delays between each instruction
// The current LCD address is read at same time but not used in this program

#include <avr/io.h>
#include <util/delay.h>
#include "../lib/lcd.h"

//LCD_getaddress reads the address counter and busy flag. For the address only,
//mask off bit7 of the return value.
unsigned char LCD_getaddr(void)
{
	//make var for the return value
	unsigned char address;
	
	PORTC &= ~((1<<LCD_RS)|(1<<LCD_E));  //RS low for busy flag read and enable low

	//RD pin low (for enable circuit)
	PORTG &= ~(1<<LCD_RD);
	// WR connected to read/write pin - high for read
	PORTG |= (1<<LCD_WR);
	//PortA is input
	DDRA = 0;
	PORTC |= (1<<LCD_E);  // enable high
	asm volatile ("nop");  // short delay
	asm volatile ("nop");
	//while E is high, get data from LCD
	address = PINA;
	
	PORTC &= ~(1<<LCD_E);   // E low (for strobe)
	// restore other settings
	PORTG |= (1<<LCD_RD);   // RD pin high (for enable circuit)
	PORTG &= ~(1<<LCD_WR);   // WR read/write goes low for write
	DDRA = 0xFF; //back to output
	//return address and busy flag
	return address;
}


//LCD_wait reads the address counter (which contains the busy flag) and loops until
//the busy flag is cleared.
void LCD_wait(void)
{
	//get address and busy flag
	//and loop until busy flag (bit7) cleared
	while((LCD_getaddr() & 0x80) == 0x80)
	{
	}
	
}



//LCD_putchar writes a character to the LCD at the current address,
//usage: LCD_putchar('A'); or LCD_putchar(0x55);
void LCD_putchar(unsigned char data)
{
	LCD_wait();  // is it busy?
	//PortA is output
	DDRA = 0xFF;

	//RS high for data, Enable high
	PORTC |= ((1<<LCD_RS)|(1<<LCD_E));
	//put data on bus
	PORTA = data;
	//the number of nops required varies with your clock frequency, Can be altered
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");

	// Enable low again
	PORTC &= ~(1<<LCD_E);
	//release bus
	DDRA = 0;
}
//LCD_command works EXACTLY like LCD_putchar, but takes RS low for accessing the command reg

void LCD_command(unsigned char command)
{
	
	LCD_wait();  // is it still busy?
	DDRA = 0xFF;  // data port as output
	
	PORTC &= ~(1<<LCD_RS);  //RS low for Command
	PORTC |= (1<<LCD_E);    //Enable pin high
	PORTA = command;         // put data on Port
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	PORTC &= ~(1<<LCD_E);   //Enable Pin Low
	DDRA = 0;                // release bus

}

/*LCD_init initialises the LCD with the following paramters:
8 bit mode, 5*7 font, 2 lines (also for 4 lines)
auto-inc cursor after write and read
cursor and display on, cursor blinking.
*/
void LCD_init(void)
{
	//setup the LCD control signals on PortC  (RS and Enable as output)
	DDRC |= ((1<<LCD_RS)|(1<<LCD_E));
	PORTC &= ~((1<<LCD_RS)|(1<<LCD_E));      // set both low
	//setup the LCD control signals on PortD
	DDRG |= ((1<<LCD_RD)|(1<<LCD_WR));          // read and write pins (STK300 on PortG)
	PORTG &= ~(1<<LCD_WR);   					// write pin low
	// data Port A as output
	DDRA = 0xFF;
	//if called right after power-up, we'll have to wait a bit (fine-tune for faster execution)
	_delay_ms(50);
	
	LCD_command(LCD_CLR);       // clear display
	LCD_command(LCD_8BIT);      // set 8 data bits
	LCD_command(LCD_INC);       // cursor increments automatically
	LCD_command(LCD_MOV);       // cursor
	//LCD_command(LCD_ALL);     // can call all instead of next line
	LCD_command(LCD_ON | LCD_ON_DISPLAY | LCD_ON_CURSOR | LCD_ON_BLINK);
	LCD_command(LCD_LINE1); // set cursor to  row 1, position 0
}


// function for showing strings on the LCD. It uses the low-level
//functions above. usage example: LCD_write("Hello World!");
void LCD_write(char* dstring)
{
	//is the character pointed at by dstring a zero? If not, write character to LCD
	while(*dstring)
	{
		// write the character from dstring to the LCD, then post-inc the dstring pointer.
		LCD_putchar(*dstring++);
	}
}

/*Note: As LCD has a RAM memory, it will display what was written until its power is removed
hence no loop needed */
