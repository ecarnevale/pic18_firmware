/*
 * muin.h
 *
 * Main variables and configuration parameters for MuIN
 * from DROIDs (994.005)
 *
 * Emanuel Carnevale 2008
 *
 */

#include <p18cxxx.h>   					// General init

#pragma config OSC		= HSPLL			// 10MHz e PPL 4x
#pragma config WDT		= OFF
#pragma config PWRT		= ON
#pragma config LVP		= OFF
#pragma config BOREN	= ON
#pragma config BORV		= 1
#pragma config XINST	= OFF
#pragma config DEBUG	= OFF

#define LC	PORTAbits.RA4				// Led com
