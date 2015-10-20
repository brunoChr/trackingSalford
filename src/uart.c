#include "../lib/uart.h"


static FILE mystdout = FDEV_SETUP_STREAM(uart_printf, NULL, _FDEV_SETUP_WRITE);


void uart_init(LONG baud)
{
	unsigned int ubrr = F_CPU/16/baud - 1;
	stdout = &mystdout;	// Setup our stdio stream.

	// [+]Set baud rate.
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t)ubrr;

	// [+]Enable receiver and transmitter.
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);

	// [+]Set frame format: 8data, 1stop bit.
	UCSR0C = (1<<USBS0) | (3<<UCSZ00);
}

void uart_putchar(CHAR data)
{
	while(!(UCSR0A & (bit(UDRE0)))); // Wait for empty transmit buffer.
	UDR0 = data;				    // Start transmission.
}

CHAR uart_getchar(void)
{
	while(!(UCSR0A & (bit(RXC0)))); // Wait for incoming data.
	return(UDR0);				   // Return the data.
}

BOOL uart_kbhit(void)
{
	// [+]Return nonzero if char waiting polled version.
	if(UCSR0A & (1<<RXC0)){
	return(TRUE);
	uart_putchar('C');
	}
	return(FALSE);
}

void uart_pstr(CHAR *s)
{
	// [+]Loop through entire string.
	while(*s)
	{
		uart_putchar(*s);
		s++;
	}

	//int size = sizeof(s)/sizeof(s[0]);
	//for(int i=0 ; i<size ; i++)
	//uart_putchar(s[i]);
}

// [+]This function is called by printf as a stream handler
INT uart_printf(CHAR var, FILE *stream)
{
	// [+]Translate \n to \r for br@y++ terminal.
	if(var == '\n')
	uart_putchar('\r');
	uart_putchar(var);
	return(FALSE);
}

void USART_Flush( void )
{
	unsigned char dummy;
	while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
}
