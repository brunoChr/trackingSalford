#ifndef UART_H
#define UART_H

#include <avr/io.h>
#include <stdio.h>

#include "cpu.h"
#include "port.h"
#include "types.h"
#include "generics.h"

void uart_init(LONG baud);
void uart_pstr(CHAR *s);
CHAR uart_getchar(void);
void uart_putchar(CHAR data);
BOOL uart_kbhit(void);
INT  uart_printf(CHAR var, FILE *stream);
void USART_Flush( void );

#endif
