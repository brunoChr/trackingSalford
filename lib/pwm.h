/*
 * pwm.h
 *
 * Created: 05/10/2015 15:19:15
 *  Author: b.christol
 */ 


#ifndef PWM_H_
#define PWM_H_

/*** DEFINE DEFINITION ***/

#define TRUE 1
#define FALSE 0
#define ANGLE 0
#define DUREE_ETAT_HAUT 1
/*** Globalvar ***/
SHORT pos;

/*** PROTOTYPE ***/

void pwm_activeInterrupt();
void pwm_init();
void pwm_rotationGauche(void);
void pwm_rotationDroite(void);
void pwm_positionCentrale(void);
void pwm_setPosition(unsigned int angle);
unsigned int pwm_getPosition(int typeSortie);

unsigned int tableDeCalcul (unsigned int angle);


#endif /* PWM_H_ */