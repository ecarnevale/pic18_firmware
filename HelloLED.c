/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 *  First attempts to use PIC18F2520 resources
 *
 *  (c) 2008 Emanuel Carnevale
 *
 *  Compiler C18
 *
 *  PIC18F2520 clock 40MHz (quarzo da 10MHz con PLL 4X attivo)
 *
 */ 

// include
#include <p18cxxx.h>   					// General init
#include <delays.h>
#include <timers.h>
#include "HelloLED.h"

// config fuse
#pragma config OSC		= HSPLL			// Usare con quarzo da 10MHz e PPL 4x


#define LC	PORTAbits.RA4				// Led 


void main()
{

// interrupt
INTCON = 0;						// all Interrupts disabled

for (S1=0;S1<100;S1++)					// LED will blink
 {
  LC=!LC;
  Delay10KTCYx(8*S1);
 }

} //main

