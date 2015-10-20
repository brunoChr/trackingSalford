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
#define TURN_LEFT 1
#define TURN_RIGHT 2


/*** Local variable stock in flash ***/
static const __flash UINT angleToValue[181] =
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