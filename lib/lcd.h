/*
 * lcd.h
 *
 * Created: 13/10/2015 16:33:49
 *  Author: b.christol
 */ 


#ifndef LCD_H_
#define LCD_H_



// used pins on port C and D
#define LCD_E  7    // PORTC.7
#define LCD_RS 6    //PORTC.6
#define LCD_RD 1    // PORTG.1
#define LCD_WR 0    // PORTG.0


// Data and Busy flag are on PORTA

//LCD commands
#define LCD_CLR 0x01     // clear LCD
#define LCD_8BIT 0x38    // 8-bit mode
#define LCD_INC 0x06    //Increment, display freeze
#define LCD_MOV 0x10    //Cursor move, not shift
#define LCD_ALL  0x0F    // LCD On, LCD display on, cursor on and blink on
#define LCD_ON                0x08      // turn lcd/cursor on
#define LCD_ON_DISPLAY        0x04      // turn display on
#define LCD_ON_CURSOR         0x02      // turn cursor on
#define LCD_ON_BLINK          0x01      // cursor blink
#define LCD_LINE1  0x80  // cursor Pos on line 1 (or with column)
#define LCD_LINE2  0xC0  // cursor Pos on line 2 (or with column)


/*** Prototype functions ***/
unsigned char LCD_getaddr(void);
void LCD_wait(void);
void LCD_putchar(unsigned char data);
void LCD_command(unsigned char command);
void LCD_init(void);
void LCD_write(char* dstring);


#endif /* LCD_H_ */