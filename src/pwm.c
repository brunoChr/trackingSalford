/*
 * pwm.c
 *
 * Created: 05/10/2015 15:21:10
 *  Author: b.christol
 */ 

#include "../lib/pwm.h"
#include <avr/interrupt.h>

void pwm_activeInterrupt()
{
	/*
	* Activation/Désactivation des interruptions
	* liées au PWM
	*/
	TIMSK |= (1 << OCIE1A);// Interrupt on compare match A enabled
	TIMSK |= (1 << TOIE1); // Interrupt on timer overflow enable
}

void pwm_init()
{
	/*
	* Initialisation du PWM
	*/
	//__enable_interrupt();
	sei(); // Activation des interruptions globales
	
	pwm_activeInterrupt();
	pwm_positionCentrale();
	
	/*Utilisation du port B*/
	DDRB |= (1 << DDB5);// PORTB5 en sortie
	PORTB |= (1 <<PORTB5); // PORTB5 active High
			
	/*Toggle OC1A on compare match*/
	TCCR1A |= (0 << COM1A1);
	TCCR1A |= (1<< COM1A0);
	
	/*Fast PWM Mode, 10-bit
	* Valeur de TOP pour l'overflow = 1023
	*/
	TCCR1A |= (1 << WGM11);
	TCCR1A |= (1 << WGM10);
	TCCR1B |= (1 << WGM13);
	TCCR1B |= (1 << WGM12);
	
	/*Clk (Prescaler : 64)*/
	TCCR1B |= (0 << CS12);
	TCCR1B |= (1 << CS11);
	TCCR1B |= (1 << CS10);
	
	/*Reglage de la valeur du timer1 à 0; 
	(Attention : empeche la comparaison jusqu'au prochain tour de clock)
	TCNT1H = 0;
	TCNT1L = 0;
	*/
	
	/*Reglage du temps haut à 1,5 ms (position centrale)*/
	/*
	OCR1AH = 0x00;
	OCR1AL = 0xC0;
	*/
}


void pwm_rotationGauche(void)
{
	/*Reglage du temps haut à 1 ms (position gauche)*/
	OCR1AH = 0x00;
	OCR1AL = 0x80;
}

void pwm_rotationDroite(void)
{
	/*Reglage du temps haut à 2 ms (position droite)*/
	OCR1AH = 0x01;
	OCR1AL = 0x00;
}

void pwm_positionCentrale(void)
{
	OCR1AH = 0x00;
	OCR1AL = 0xC0;
}
