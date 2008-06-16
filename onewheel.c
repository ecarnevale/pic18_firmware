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
 *  Compilare con C18 v3.0 o superiore
 *  il programma � compatibile con la versione student 
 *
 *  PIC18F2520 clock 40MHz (quarzo da 10MHz con PLL 4X attivo)
 *
 */ 

/*
EEPROM default

0x00 Carrier PWM
0x01 Duty PWM
0x02 Baud rate
0x03 GPIO/SERVO
0x04 PORTB dir 

PWM carrier
2 = 2.44 kHz
1 = 9.76 kHz
0 = 39.44 kHz

PWM duty
0 = 0%
1 = 50%
2 = 100%

Baud rate
0 = 115200
1 = 57600
2 = 38400
3 = 19200
4 = 9600
5 = 4800
6 = 2400
7 = 1200

TRISB
0 = 8 GPIO
1 = 4 GPIO (RB4-RB7, 4 servo (RB0-RB3)
2 = 8 Servo
*/

// preinit EEPROM durante la programmazione
#pragma romdata eedata_scn=0xf00000
	rom char eedata_values[5] = {0x01,0x01,0x00,0x01,0xF0};
#pragma romdata

//**** inizio header programma ****
// include
#include <p18cxxx.h>   					// General init
#include <delays.h>
#include <timers.h>
#include <stdlib.h>
#include <usart.h>	
//#include <adc.h>
//#include <pwm.h>
//#include <i2c.h>
#include "onewheel.h"

// config fuse
#pragma config OSC		= HSPLL			// Usare con quarzo da 10MHz e PPL 4x
#pragma config WDT		= OFF
#pragma config PWRT		= ON
#pragma config LVP		= OFF
#pragma config BOREN	= ON
#pragma config BORV		= 1
#pragma config XINST	= OFF
#pragma config DEBUG	= OFF


#define LC	PORTAbits.RA4				// Led com

char Ssel,SGr1=0,SGr2=0,ParsFlag=0;
int PWM1,PWM2;
int S1,S2,S3,S4,S5,S6,S7,S8;
int ADC1,ADC2,ADC3,ADC4,ADC5,SRFdist,Compass;
char str[6],strcom[10],TXcount,AT_flag;
unsigned char lettura[2];

/**** fine header programma ****/

#pragma code high_vector=0x08			// vettore interrupt
void interrupt_at_high_vector(void)
{
	_asm GOTO ISRgest _endasm			// salta alla funzione per interrupt
}
#pragma code							// ritorna al codice C
#pragma interrupt ISRgest

void main()
{
LATA = 0x00;
LATB = 0x00;
LATC = 0x00;		

TRISA = 0b11101111;						// input analogici, RA4 GPIO		
TRISB = 0xFF;							// Servi e GPIO
TRISC = 0b11011000;						// I2C, Seriale, PWM

// interrupt
INTCON = 0;								// disattiva tutti gli interrupt
INTCONbits.GIE = 1;						// attiva interrupt UART e Timer
INTCONbits.PEIE = 1;

for (S1=0;S1<100;S1++)					// fa lampeggiare il LED
 {
  LC=!LC;
  Delay10KTCYx(80);
 }

/* Init periferiche */
init_sys();								// legge parametri di default.

while(1)
 {
 putrsUSART("Hello world!\n");
 }

}//main

void ISRgest(void)		// I.S.R.
{
char data;				// buffer ricezione 

}


void init_sys(void)
{
 char tmp;
 int tmp2;
 
 // setup PWM Carrier 
 tmp = EEPROMREAD(0x00);
 if (tmp==0) OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1);	// 39.06 kHz
 if (tmp==1) OpenTimer2(TIMER_INT_OFF & T2_PS_1_4 & T2_POST_1_1);	//  9.77 kHz
 if (tmp==2) OpenTimer2(TIMER_INT_OFF & T2_PS_1_16 & T2_POST_1_1);	//  2.44 kHz			

 // setup PWM duty
 tmp = EEPROMREAD(0x01);
 if (tmp==0)
  {
   PWM1=1;
   PWM2=1;
  }
 if (tmp==1)
  {
   PWM1=512;
   PWM2=512;
  }
 if (tmp==2)
  {
   PWM1=1022;
   PWM2=1022;
  } 
 
 // usart
 tmp = EEPROMREAD(0x02);
 if (tmp==0) tmp2 = 86;
 if (tmp==1) tmp2 = 173;
 if (tmp==2) tmp2 = 259;
 if (tmp==3) tmp2 = 520;
 if (tmp==4) tmp2 = 1041;
 if (tmp==5) tmp2 = 2082;
 if (tmp==6) tmp2 = 4166;
 if (tmp==7) tmp2 = 8332;

 BAUDCONbits.BRG16 = 1;						// baud rate generator a 16 bit
 OpenUSART (USART_TX_INT_OFF &		
			USART_RX_INT_ON &										  
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_HIGH, 1041);	
//			USART_BRGH_HIGH, tmp2);	
/*
Con BRGH = 1, BRG = 16 (risoluzione a 16 bit)
1200   = 8332
2400   = 4166
4800   = 2082
9600   = 1041
19200  = 520
38400  = 259
57600  = 173
115200 = 86
*/

 // setup PORTB
 tmp = EEPROMREAD(0x03);
 if (tmp==0)				// all GPIO
  {
   TRISB = 0xFF;
   SGr1=0;
   SGr2=0; 
  }

 if (tmp==1)				// 4 GPIO, 4 Servo
  {
   TRISB = 0xF0;
   SGr1=1;
   SGr2=0; 
 }

 if (tmp==2)				// all Servo
  {
   TRISB = 0x00;
   SGr1=1;
   SGr2=1;
  }
  
 tmp = EEPROMREAD(0x04);	// controllo direzione
 TRISB = tmp;
}

void EEPROMWRITE(char Data, char Adr)
{
EEADR = Adr;
EEDATA= Data;
EECON1bits.EEPGD = 0;		// Punta alla EEPROM
EECON1bits.CFGS  = 0;		// Permette uso EEPROM
EECON1bits.WREN = 1;		// abilita scrittura EEPROM	
INTCONbits.GIE  = 0;		// disabilita interrupt
EECON2 = 0x55;				// inizializza ciclo di scrittura
EECON2 = 0xAA;
EECON1bits.WR = 1;			// attiva ciclo di scrittura, circa 4ms
while (EECON1bits.WR);		// attende termine scrittura
EECON1bits.WREN = 0;		// disabilita scrittura EEPROM
INTCONbits.GIE = 1;			// riattiva interrupt
}

char EEPROMREAD(char Adr)
{
EEADR = Adr;
EECON1bits.EEPGD = 0;		// Punta alla EEPORM
EECON1bits.CFGS  = 0;		// Permette uso EEPROM
EECON1bits.RD = 1;			// Legge locazione, 1 ciclo macchina
return EEDATA;
}

