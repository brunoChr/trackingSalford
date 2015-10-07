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
	* Activation/D�sactivation des interruptions
	* li�es au PWM
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
	PORTB |= (1 << PORTB5); // PORTB5 active High
			
	/*Toggle OC1A on compare match*/
	TCCR1A |= (1 << COM1A1);
	TCCR1A |= (0 << COM1A0);
	
	/*Fast PWM Mode, 10-bit
	* Valeur de TOP pour l'overflow = 1023
	*/
	TCCR1A |= (0 << WGM11);
	TCCR1A |= (1 << WGM10);
	TCCR1B |= (1 << WGM13);
	TCCR1B |= (1 << WGM12);
	
	/*Clk (Prescaler : 64)*/
	TCCR1B |= (0 << CS12);
	TCCR1B |= (1 << CS11);
	TCCR1B |= (1 << CS10);
	
}


void pwm_rotationGauche(void)
{
	/*Reglage du temps haut � 1 ms (position extreme gauche)*/
	OCR1AH = 0x00;
	OCR1AL = 0x80;
}

void pwm_rotationDroite(void)
{
	/*Reglage du temps haut � 2 ms (position extreme droite)*/
	OCR1AH = 0x01;
	OCR1AL = 0x00;
}

void pwm_positionCentrale(void)
{
	OCR1AH = 0x00;
	OCR1AL = 0xC0;
}

void pwm_setPosition(double angle)
{
	/*
	 Controle de la rotation en PWM

	Avec un prescaler de 64, la periode est de 8 ms.
	Le Fast PWM mode est cod� sur 10 bits ; on peut donc aller de zero � 1023, en 8 ms.
	Duree de l'�tat haut : 
	- Position extreme gauche = 1ms ; 
	- Position centrale = 1,5 ms ; 
	- Position extreme droite = 2 ms
	
	Si on convertit en nombre de bit, l'extreme gauche � 128 ; la position centrale est � 192 ; l'extreme droite � 256
	Ces positions correspondent respectivement � -90 , 0 , et +90�
	Donc, pour parcourir 90�, on compte 64 coups d'horloe (256 - 192, 192 - 128)
	On convertit nos degr�s en nombre de coups d'horloge en divisant 64 par 90, et en multipliant la valeur obtenue
	par angle que l'on souhaite
	
	*/	
	double degreToBit = 64/90;
	
	//Gestion des valeurs d�passant les valeurs extremes
	if(angle >= 90)
	{
		pwm_rotationDroite();
	}
	else if (angle <= (-90))
	{
		pwm_rotationGauche();
	}
	
	//Gestion des valeurs comprise dans l'intervalle utile
	else
	{
		int valeur = angle *  degreToBit;
		
		if(angle < 0)
		{
			
			OCR1AH = 0x00;
			OCR1AL = 0xC0 - valeur;
		}
		else if (angle > 0)
		{
			OCR1AH = 0x00;
			OCR1AL = 0xC0 + valeur;
		}
	}
	
	/*
	//Cas 1 : angle  positif (rotation vers la droite)
	if (angle >= 0)
	{
		if (angle == 0)
			pwm_positionCentrale();
		else if(angle > 0 && angle <= 15)
		{
			OCR1AH = 0x00;
			OCR1AL = 0xCA;
		}
		else if(angle > 15 && angle <= 30)
		{
			OCR1AH = 0x00;
			OCR1AL = 0xD4;
		}
		else if(angle > 30 && angle <= 45)
		{
			OCR1AH = 0x00;
			OCR1AL = 0xDE;
		}
		else if(angle > 45 && angle <= 60)
		{
			OCR1AH = 0x00;
			OCR1AL = 0xE8;
		}
		else if(angle > 60 && angle <= 75)
		{
			OCR1AH = 0x00;
			OCR1AL = 0xF2;
		}
		else if(angle > 75 && angle <= 90)
		{
			OCR1AH = 0x00;
			OCR1AL = 0xFC;
		}
		else if(angle > 90)
			pwm_rotationDroite();
	}
	
	//Cas 2 : angle n�gatif (rotation vers la gauche)
	else 
	{
		if(angle >= (-15) && angle < 0)
		{
			OCR1AH = 0x00;
			OCR1AL = 0xB6;
		}
		else if(angle >= (-30) && angle < (-15))
		{
			OCR1AH = 0x00;
			OCR1AL = 0xAC;
		}
		else if(angle >= (-45) && angle < (-30))
		{
			OCR1AH = 0x00;
			OCR1AL = 0xA2;
		}
		else if(angle >= (-60) && angle < (-45))
		{
			OCR1AH = 0x00;
			OCR1AL = 0x98;
		}
		else if(angle >= (-75) && angle < (-60))
		{
			OCR1AH = 0x00;
			OCR1AL = 0x8E;
		}
		else if(angle > (-90) && angle < (-75))
		{
			OCR1AH = 0x00;
			OCR1AL = 0x84;
		}
		else if(angle < (-90))
			pwm_rotationGauche();
	}
	*/
}

