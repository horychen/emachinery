/* =================================================================================
File name:       PI.H 
===================================================================================*/


#ifndef __PI_H__
#define __PI_H__

#include "typedef.h"


typedef struct {  REAL  Ref;   			// Input: reference set-point
				  REAL  Fbk;   			// Input: feedback
				  REAL  Out;   			// Output: controller output 
				  REAL  Kp;				// Parameter: proportional loop gain
				  REAL  Ki;			    // Parameter: integral gain
				  REAL  Umax;			// Parameter: upper saturation limit
				  REAL  Umin;			// Parameter: lower saturation limit
				  REAL  up;				// Data: proportional term
				  REAL  ui;				// Data: integral term
				  REAL  v1;				// Data: pre-saturated controller output
				  REAL  i1;				// Data: integrator storage: ui(k-1)
				  REAL  w1;				// Data: saturation record: [u(k-1) - v(k-1)]
				} PI_CONTROLLER;


/*-----------------------------------------------------------------------------
Default initalisation values for the PI_GRANDO objects
-----------------------------------------------------------------------------*/                     

#define PI_CONTROLLER_DEFAULTS {		\
						   0, 			\
                           0, 			\
						   0, 			\
                           1.0,	\
                           0.0,	\
                           1.0,	\
                           -1.0, 	\
                           0.0,	\
                           0.0, 	\
                           0.0,	\
                           0.0,	\
                           1.0 	\
              			  }


/* 陈艺铭找到的德州仪器官方PI */
#define USE_LAOMING_PI FALSE /* change PI algorithm*/
extern PI_CONTROLLER texas_pi_id;
extern PI_CONTROLLER texas_pi_iq;
extern PI_CONTROLLER texas_pi_spd;

/*------------------------------------------------------------------------------
	PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/
#define   _IQsat(A, Pos, Neg)  (fmaxf(((fminf((A),(Pos)))),(Neg)))
#define PI_MACRO(v)												\
																\
	/* proportional term */ 									\
	v.up = (v.Kp * (v.Ref - v.Fbk));						\
																\
	/* integral term */ 										\
	v.ui = (v.Out == v.v1)?( (v.Ki * v.up)+ v.i1) : v.i1;	\
	v.i1 = v.ui;												\
																\
	/* control output */ 										\
	v.v1 = v.up + v.ui;											\
	v.Out= _IQsat(v.v1, v.Umax, v.Umin);						\
	//v.w1 = (v.Out == v.v1) ? _IQ(1.0) : _IQ(0.0);				\

// ***********************************************************************************
//   This macro works with angles as inputs, hence error is rolled within -pi to +pi
// ***********************************************************************************
#define PI_POS_MACRO(v)										    \
	/* proportional term */ 									\
	v.up = v.Ref - v.Fbk;										\
	if (v.up >= (0.5))  										\
	  v.up -= (1.0); 			/* roll in the error */	    \
	else if (v.up <= (-0.5))  								\
	  v.up += (1.0); 	        /* roll in the error */	    \
																\
	/* integral term */ 										\
	v.up = (v.Kp * v.up);									\
	v.ui = (v.Out == v.v1)?((v.Ki * v.up)+ v.i1) : v.i1;	\
	v.i1 = v.ui;												\
																\
	/* control output */ 										\
	v.v1 = v.up + v.ui;								            \
	v.Out= _IQsat(v.v1, v.Umax, v.Umin);						\
	//v.w1 = (v.Out == v.v1) ? _IQ(1.0) : _IQ(0.0);				\


#endif // __PI_H__

