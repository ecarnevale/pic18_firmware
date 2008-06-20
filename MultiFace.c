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
 *  Multi Interface
 * 
 *  (c) 2008 By DROID s.a.s.
 *  developed by Marco d'Ambrosio
 *
 *  Compilare con C18 v3.0 o superiore
 *  il programma è compatibile con la versione student 
 *
 *  PIC18F2520 clock 40MHz (quarzo da 10MHz con PLL 4X attivo)
 *
 *  Versione 1.6.0 build 14/02/2008
 * 
 *  Interfacce gestite:
 *  
 *  I2C, read e write operation 
 *  ADC, cinque canali a dieci bit
 *  PWM, due canali con direzione 
 *  GPIO, 0-4-8 bit sia In che OUT
 *  RC Servo, 0-4-8 servo RC da -90° a +90° risoluzione 1us
 *  Comunicazione wireless tramite modulo Xbee a 115200 bps
 *  Comunicazione seriale TTL tra 1200 e 115200 bps
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
#include <adc.h>
#include <pwm.h>
#include <i2c.h>
#include "MultiFace.h"

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

// Timer
OpenTimer0( TIMER_INT_OFF & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1);		// servo frame
OpenTimer1(TIMER_INT_ON & T1_PS_1_1 & T1_SOURCE_INT & T1_OSC1EN_OFF);	// Servo pulse 1
OpenTimer3(TIMER_INT_ON & T3_PS_1_1 & T3_SOURCE_INT & T3_OSC1EN_OFF );	// Servo Pulse 2

// PWM
OpenPWM1(0xff);			 
OpenPWM2(0xff);

SetDCPWM1(PWM1);		
SetDCPWM2(PWM2);

// ADC
ADCON1 = 0b00001010;		// AN0, AN1, AN2, AN3,AN4 attivi
ADCON2 = 0b10110010;		// 16 TAD, Fosc/32
ADCON0 = 0x01;				// ADC ON

//I2C
OpenI2C(MASTER, SLEW_OFF);	// Attiva bus I2C, Master mode 100 kbits
SSPADD =0x63; 				//400kHz clock(19H) @40MHz
    						//100kHz clock(63H) @40MHz (default)

// Azzeramento Timer 
T1CONbits.TMR1ON = 0;
T3CONbits.TMR3ON = 0;

TMR3H=TMR3L=0;
TMR1H=TMR1L=0; 

T0CONbits.TMR0ON = 1;
T1CONbits.TMR1ON = 1;
T2CONbits.TMR2ON = 1;
T3CONbits.TMR3ON = 1;

S1=S2=S3=S4=1500;
S5=S6=S7=S8=1500;
Ssel = 1;

LC   = 1;
AT_flag = 1;
TXcount = 0;

while(1)
 {
 // if (INTCONbits.TMR0IF)
 //  {
 //   ServoCiclo();			// gestione servi
 //   LC=1;
 //  }
  
 // ADCCiclo();  
 // SetDCPWM1(PWM1);			// PWM1 e PWM2
 // SetDCPWM2(PWM2);

 // if(ParsFlag) Parser();	// analisi comandi
 //}
 // scrittura UART
 WriteUSART("Greetings from the MuIN!\r\n");
 }
}

/* ---------- fime main ---------- */ 



void ISRgest(void)		// I.S.R.
{
char data;				// buffer ricezione 

 if (PIR1bits.TMR1IF)
  {
   if (SGr1)
    {
     LATBbits.LATB0 = 0;
     LATBbits.LATB2 = 0;
    }
   if (SGr2)
	{
     LATBbits.LATB4 = 0;
     LATBbits.LATB6 = 0;
    }
   
   T1CONbits.TMR1ON = 0; 
   PIR1bits.TMR1IF = 0;
  }

if (PIR2bits.TMR3IF)
  {
   if (SGr1)
    {
     LATBbits.LATB1 = 0;
     LATBbits.LATB3 = 0;
    }  
   if (SGr2)
    {
     LATBbits.LATB5 = 0;
     LATBbits.LATB7 = 0;
    }
   
   T3CONbits.TMR3ON = 0; 
   PIR2bits.TMR3IF = 0;
  }

if (PIR1bits.RCIF)
 {
  data = RCREG;
 
  if (data == '@' & AT_flag) TXcount = 0, AT_flag = 0;
  strcom[TXcount] = data;
  TXcount++;
  
  if (TXcount == 8 & data == '#')
   { 
    TXcount = 0, ParsFlag = 1;	
    AT_flag = 1;
    LC = 0;
   }
  
  if (TXcount > 7) AT_flag = 1;
 }
}

void SRFping(unsigned char ADDS)
{
EEByteWrite(ADDS, 0x00, 0x51);	// Start ranging, risultato in cm.
EEAckPolling(ADDS);				// attende per ACK
}

void SRFread(unsigned char ADDS)
{
EESequentialRead(ADDS,0x02,lettura,0x02);
EEAckPolling(ADDS);
SRFdist = lettura[0]*256+lettura[1];
}

void CMPS03(unsigned char ADDS)
{
EESequentialRead(ADDS,0x02,lettura,0x02);
EEAckPolling(ADDS);

Compass = lettura[0]*256+lettura[1];
}

void MD22(char ADDS, char M1, char M2)
{
char mm2,mm1;

mm1 = M1;
mm2 = M2;

if (M2 == 1)
 {
  EEByteWrite(ADDS, 0x01, M1);	// imposta velocità M1
  EEAckPolling(ADDS);			// attende per ACK
 } 

if (M2 == 2) 
 {
  EEByteWrite(ADDS, 0x02, M1);	// imposta velocità M2
  EEAckPolling(ADDS);			// attende per ACK
 }
}

void MD22mod (char ADDS, char M1)
{
 EEByteWrite(ADDS, 0x00, M1);	// imposta modo
 EEAckPolling(ADDS);			// attende per ACK

 EEByteWrite(ADDS, 0x01, 0x80);	// imposta velocità M1
 EEAckPolling(ADDS);			// attende per ACK
 
 EEByteWrite(ADDS, 0x02, 0x80);	// imposta velocità M2
 EEAckPolling(ADDS);			// attende per ACK
}

void I2CW()
{
 char ADDS,N1,d1,d2,d3;

 N1 = strcom[2]; 
 ADDS = strcom[3]; 
 d1 = strcom[4]; 
 d2 = strcom[5]; 
 d3 = strcom[6]; 

  if (N1 == 1)
  {    
   EEByteWrite(ADDS, 0,d2);		// Write byte
   EEAckPolling(ADDS);			// attende per ACK
  }

 if (N1 == 2)
  {    
   EEByteWrite(ADDS, d1,d2);	// Write byte
   EEAckPolling(ADDS);			// attende per ACK
  }

 if (N1 == 3)
  {    
   EEByteWrite(ADDS,d1,d2);		// Write byte
   EEAckPolling(ADDS);			// attende per ACK
    
   EEByteWrite(ADDS,(d1+1),d2);	// Write byte
   EEAckPolling(ADDS);			// attende per ACK
  }
}

void I2CR()
{
 char ADDS,N1,d1,d2,i;
 unsigned char let_b[16];

 N1 = strcom[3]; 	// numero byte da leggere
 ADDS = strcom[2]; 	// I2C Address
 d1 = strcom[4];	// option

 EESequentialRead(ADDS,d1,let_b,N1);
 EEAckPolling(ADDS);

 TXREG = '@';
 while(!TXSTAbits.TRMT); 
 TXREG = 'I';
 while(!TXSTAbits.TRMT); 
 TXREG = ADDS;
 while(!TXSTAbits.TRMT); 

for (i=0;i<N1;i++)
  {
   TXREG = let_b[i];
   while(!TXSTAbits.TRMT); 
  }
}

void GPIO_read(void)
{
 TXREG = '@';
 while(!TXSTAbits.TRMT); 
 TXREG = 'G';
 while(!TXSTAbits.TRMT); 
  
 TXREG = PORTB;
 while(!TXSTAbits.TRMT); 
		
 TXREG = 0;
 while(!TXSTAbits.TRMT);
 TXREG = 0;
 while(!TXSTAbits.TRMT);
 TXREG = 0;
 while(!TXSTAbits.TRMT);
 TXREG = 0;
 while(!TXSTAbits.TRMT);
 
 TXREG = '#';
 while(!TXSTAbits.TRMT); 
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

void SRF_Ad_Ch(char A1, char A2)
{
 EEByteWrite(A1,0,0xA0);	// Write byte
 EEAckPolling(A1);			// attende per ACK

 EEByteWrite(A1,0,0xAA);	// Write byte
 EEAckPolling(A1);			// attende per ACK
 
 EEByteWrite(A1,0,0xA5);	// Write byte
 EEAckPolling(A1);			// attende per AC

 EEByteWrite(A1,0,A2);		// Write byte
 EEAckPolling(A1);			// attende per ACK
}

void CPMS03_Ad_Ch(char A1, char A2)
{
 EEByteWrite(A1,12,0xA0);	// Write byte
 EEAckPolling(A1);			// attende per ACK

 EEByteWrite(A1,13,0xAA);	// Write byte
 EEAckPolling(A1);			// attende per ACK
 
 EEByteWrite(A1,14,0xA5);	// Write byte
 EEAckPolling(A1);			// attende per AC

 EEByteWrite(A1,15,A2);		// Write byte
 EEAckPolling(A1);			// attende per ACK
}