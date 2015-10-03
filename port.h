#ifndef PORT_H
#define PORT_H

#include <avr/io.h>

#define clr(port, pin)		(PORT ## port &= ~(1<<pin))
#define set(port, pin)		(PORT ## port |=  (1<<pin))
#define tog(port, pin)		(PORT ## port ^=  (1<<pin))
#define get(port, pin)		(PIN  ## port &   (1<<pin))
#define out(port, pin)		(DDR  ## port |=  (1<<pin))
#define inp(port, pin)		(DDR  ## port &= ~(1<<pin))
#define chb(reg,  bt)		(reg & (1<<bt))
#define bit(x)				_BV(x)

#define LED(x)				(set(B, x))
#define BUTTON(x)			(get(D, x))

void port_init(void);

#endif
