/*!
 *  \file pwm.c
 *
 *  Created: 05/10/2015 15:21:10
 *  Author: b.christol
 */ 

#include "../lib/pwm.h"
#include <avr/interrupt.h>

unsigned int tableDeCalcul(unsigned int angle)
{
	unsigned int angleToValue[181] = 
		{
			1000, 1006, 1012, 1018, 1024, 1030, 1036, 1042, 1048, 1054,
			1060, 1066, 1072, 1078, 1084, 1090, 1096, 1102, 1108,
			1114, 1120, 1126, 1132, 1138, 1144, 1150, 1156, 1162,
			1168, 1174, 1180, 1186, 1192, 1198, 1204, 1210, 1216,
			1222, 1228, 1234, 1240, 1246, 1252, 1258, 1264, 1270,
			1276, 1282, 1288, 1294, 1300, 1306, 1312, 1318, 1324,
			1330, 1336, 1342, 1348, 1354, 1360, 1366, 1372, 1378,
			1384, 1390, 1396, 1402, 1408, 1414, 1420, 1426, 1432,
			1438, 1444, 1450, 1456, 1462, 1468, 1474, 1480, 1486,
			1492, 1498, 1504, 1510, 1516, 1522, 1528, 1534, 1540,
			1546, 1552, 1558, 1564, 1570, 1576, 1582, 1588, 1594,
			1600, 1606, 1612, 1618, 1624, 1630, 1636, 1642, 1648,
			1654, 1660, 1666, 1672, 1678, 1684, 1690, 1696, 1702,
			1708, 1714, 1720, 1726, 1732, 1738, 1744, 1750, 1756,
			1762, 1768, 1774, 1780, 1786, 1792, 1798, 1804, 1810,
			1816, 1822, 1828, 1834, 1840, 1846, 1852, 1858, 1864,
			1870, 1876, 1882, 1888, 1894, 1900, 1906, 1912, 1918,
			1924, 1930, 1936, 1942, 1948, 1954, 1960, 1966, 1972,
			1978, 1984, 1990, 1996, 2002, 2008, 2014, 2020, 2026,
			2032, 2038, 2044, 2050, 2056, 2062, 2068, 2074, 2080
		};
		
	return angleToValue[angle];
}
void pwm_activeInterrupt()
{
	/*
	* Activation/Désactivation des interruptions
	* liées au PWM
	*/
	ETIMSK |= (1 << OCIE3A);// Interrupt on compare match A enabled
	ETIMSK |= (1 << TOIE3); // Interrupt on timer overflow enable
}

void pwm_init()
{
	/*
	* Initialisation du PWM
	*/

	sei(); // Activation des interruptions globales
	
	pwm_activeInterrupt();
	pwm_positionCentrale();
	
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
	
}


void pwm_rotationGauche(void)
{
	/*Reglage du temps haut à 1 ms (position extreme gauche)*/

	//OCR3A = ICR3/20; //20ms pour une fréquence de PWM = 50Hz
	OCR3A = tableDeCalcul(0);
}

void pwm_rotationDroite(void)
{
	/*Reglage du temps haut à 2 ms (position extreme droite)*/
	//OCR3A = ICR3 * (2/20); //20ms pour une fréquence de PWM = 50Hz
	OCR3A = tableDeCalcul(180);
}

void pwm_positionCentrale(void)
{
	//OCR3A = ICR3 * (1.5/20); //20ms pour une fréquence de PWM = 50Hz
	OCR3A = tableDeCalcul(90);
}

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
	}
	else if (angle <= (0))
	{
		pwm_rotationGauche();
	}
	//Gestion des valeurs comprise dans l'intervalle utile
	else
	{
		OCR3A = tableDeCalcul(angle);
	}

}

