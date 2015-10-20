/*!
 *  \file pwm.c
 *
 *  Created: 05/10/2015 15:21:10
 *  Author: b.christol
 */ 

#include "../lib/types.h"
#include "../lib/pwm.h"
#include <avr/interrupt.h>

/*** Local variable file ***/
SHORT pos;

/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
static UINT tableDeCalcul(UINT angle)
{		
	return angleToValue[angle];
}


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
void pwm_activeInterrupt()
{
	/*
	* Activation/Désactivation des interruptions
	* liées au PWM
	*/
	ETIMSK |= (1 << OCIE3A);// Interrupt on compare match A enabled
	ETIMSK |= (1 << TOIE3); // Interrupt on timer overflow enable
}


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
void pwm_init()
{
	/*
	* Initialisation du PWM
	*/

	//sei(); // Interrupt are enable at RTOS startup
	
	//pwm_positionCentrale();
	
	/*Utilisation du port B*/
	DDRE |= (1 << DDE3);// PORTE3 en sortie
	PORTE |= (1 << PE3); // PORTE3 active High
			
	/*Toggle OC1A on compare match*/
	TCCR3A |= (1 << COM3A1);
	TCCR3A |= (0 << COM3A0);
	
	/*Fast PWM Mode, 10-bit
	* Valeur de TOP pour l'overflow = 19 999
	*/

	ICR3 = 19999;
	//ICR3 = 2500;

	TCCR3B |= (1 << WGM33);
	TCCR3B |= (1 << WGM32);
	TCCR3A |= (1 << WGM31);
	TCCR3A |= (0 << WGM30);
	
	/*
	CSn2	CSn1	CSn0	Description
	0		0		0		No clock source. (Timer/Counter stopped)
	0		0		1		clkI/O/1 (No prescaling
	0		1		0		clkI/O/8 (From prescaler)
	0		1		1		clkI/O/64 (From prescaler)
	1		0		0		clkI/O/256 (From prescaler)
	1		0		1		clkI/O/1024 (From prescaler)
	1		1		0		External clock source on Tn pin. Clock on falling edge
	1		1		1		External clock source on Tn pin. Clock on rising edge*/
	
	/*Clk (Prescaler : 8)*/
	TCCR3B |= (0 << CS32);
	TCCR3B |= (1 << CS31);
	TCCR3B |= (0 << CS30);
	
	pwm_activeInterrupt();
}


void pwm_rotationGauche(void)
{
	/*Reglage du temps haut à 1 ms (position extreme gauche)*/

	//OCR3A = ICR3/20; //20ms pour une fréquence de PWM = 50Hz
	OCR3A = tableDeCalcul(0);
	pos = 0;
}


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
void pwm_rotationDroite(void)
{
	/*Reglage du temps haut à 2 ms (position extreme droite)*/
	//OCR3A = ICR3 * (2/20); //20ms pour une fréquence de PWM = 50Hz
	OCR3A = tableDeCalcul(180);
	pos = 180;
}


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
void pwm_positionCentrale(void)
{
	//OCR3A = ICR3 * (1.5/20); //20ms pour une fréquence de PWM = 50Hz
	OCR3A = tableDeCalcul(90);
	pos = 90;
}


/*! \fn
 *  \brief
 *  \param 
 *  \param 
 *  \exception 
 *  \return
 */
void pwm_setPosition(unsigned int angle)
{
	/*
	Controle de la rotation en PWM

	Le TOP Fast PWM mode est codé sur celui du ICR = 19999 ; on peut donc aller de zero à 19999, en 20 ms.
	Duree de l'état haut :
	- Position extreme gauche = 1ms ;
	- Position centrale = 1,5 ms ;
	- Position extreme droite = 2 ms
	Pour se placer à un angle donné on doit jouer sur la temps haut (compris entre 1ms et 2ms).
	Si on veut une résolution de 1 degré on modifie le temps haut par pas de 1/180=0,005 ms.
	*/

	//Gestion des valeurs dépassant les valeurs extremes
	if(angle >= 180)
	{
		pwm_rotationDroite();
		pos = 180;
	}
	else if (angle <= (0))
	{
		pwm_rotationGauche();
		pos = 0;
	}
	//Gestion des valeurs comprise dans l'intervalle utile
	else
	{
		OCR3A = tableDeCalcul(angle);
		pos = angle;
	}

}

UINT pwm_getPosition(char typeSortie)
{
	/*! \fn unsigned int pwm_getPosition(int typeSortie)
	*	\brief Give back the position of the servo, in degrees or in milliseconds
	*	\param typeSortie
	*	\param position : actual position of the servomotor, in degrees
	*	\exception Return -1 if we ask somthing else than angle or duration
	*	\return the new position of the servo, in degrees or milliseconds
	*/
	switch (typeSortie)
	{
		case ANGLE: //!< if the position is ask in degrees, we give back the angle position
			return pos;
			break;
		
		case DUREE_ETAT_HAUT: //!< if the position is a duration, we give back the duration of the high-state (OCR3A)
			return tableDeCalcul(pos);
			break;
		
		default:
			return -1;
			break;
	}
	return -1;
}

