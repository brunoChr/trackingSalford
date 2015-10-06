/*! \file generics.h
    \brief define generics define
    
    This is to test the documentation of defines.
*/

#ifndef GENERICS_H
#define GENERICS_H

#define	TRUE			1
#define	FALSE			0

/*!
  \def MAX(x,y)
  Computes the maximum of \a x and \a y.
*/

/*! 
   Computes the absolute value of its argument \a x.
*/
#define MIN(X, Y)		((X < Y) ? X : Y)
#define MAX(X, Y)		((X > Y) ? X : Y)
#define SIZE(x)			(sizeof(x) / sizeof(x[0]))

/*!< Computes the minimum of \a x and \a y. */
		
#endif
