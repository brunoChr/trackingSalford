/*
 * pwm.h
 *
 * Created: 05/10/2015 15:19:15
 *  Author: b.christol
 */ 


#ifndef PWM_H_
#define PWM_H_


#include "../lib/types.h"

/*** DEFINE DEFINITION ***/
#define TRUE 1
#define FALSE 0
#define ANGLE 0
#define DUREE_ETAT_HAUT 1



/*** Local variable stock in flash ***/
static const __flash UINT angleToValue[181] =
	{
		1050, 1055, 1060, 1065, 1070, 1075, 1080, 1085, 1090, 1095,
		1100, 1105, 1110, 1115, 1120, 1125, 1130, 1135, 1140,
		1145, 1150, 1155, 1160, 1165, 1170, 1175, 1180, 1185,
		1190, 1195, 1200, 1205, 1210, 1215, 1220, 1225, 1230,
		1235, 1240, 1245, 1250, 1255, 1260, 1265, 1270, 1275,
		1280, 1285, 1290, 1295, 1300, 1305, 1310, 1315, 1320,
		1325, 1330, 1335, 1340, 1345, 1350, 1355, 1360, 1365,
		1370, 1375, 1380, 1385, 1390, 1395, 1400, 1405, 1410,
		1415, 1420, 1425, 1430, 1435, 1440, 1445, 1450, 1455,
		1460, 1465, 1470, 1475, 1480, 1485, 1490, 1495, 1500,
		1505, 1510, 1515, 1520, 1525, 1530, 1535, 1540, 1545,
		1550, 1555, 1560, 1565, 1570, 1575, 1580, 1585, 1590,
		1595, 1600, 1605, 1610, 1615, 1620, 1625, 1630, 1635,
		1640, 1645, 1650, 1655, 1660, 1665, 1670, 1675, 1680,
		1685, 1690, 1695, 1700, 1705, 1710, 1715, 1720, 1725,
		1730, 1735, 1740, 1745, 1750, 1755, 1760, 1765, 1770,
		1775, 1780, 1785, 1790, 1795, 1800, 1805, 1810, 1815,
		1820, 1825, 1830, 1835, 1840, 1845, 1850, 1855, 1860,
		1865, 1870, 1875, 1880, 1885, 1890, 1895, 1900, 1905,
		1910, 1915, 1920, 1925, 1930, 1935, 1940, 1945, 1950,
	};
	
/*** 2 last line lut pilou ***/
/*1978, 1984, 1990, 1996, 2000, 2008, 2014, 2020, 2026,
2032, 2038, 2044, 2050, 2056, 2062, 2068, 2074, 2080*/

/*** Global variable ***/
extern SHORT pos;

/*** PROTOTYPE ***/
void pwm_activeInterrupt();
void pwm_init();
void pwm_rotationGauche(void);
void pwm_rotationDroite(void);
void pwm_positionCentrale(void);
void pwm_setPosition(UINT angle);
//static UINT tableDeCalcul (UINT angle);
BYTE pwm_getPosition(char typeSortie);

#endif /* PWM_H_ */