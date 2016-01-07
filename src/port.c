#include "../lib/port.h"


/*! \fn		void port_init(void)
 *  \brief	Init port
 */
void port_init(void)
{
	DDRB = (1<<PINB0) |	// LED0
		   (1<<PINB1) |	// LED1
		   (1<<PINB2) |	// LED2
		   (1<<PINB3) |	// LED3
		   (1<<PINB4) |	// LED4
		   (1<<PINB5) |	// LED5
		   (1<<PINB6) |	// LED6
		   (1<<PINB7);	// LED7

	DDRD = (0<<PIND0) |	// Button0
		   (0<<PIND1) |	// Button1
		   (0<<PIND2) |	// Button2
		   (0<<PIND3) |	// Button3
		   (0<<PIND4) |	// Button4
		   (0<<PIND5) |	// Button5
		   (0<<PIND6) |	// Button6
		   (0<<PIND7);	// Button7
}
