/*
 *	Onewheeled scooter header file
 *	
 */

// prototipi funzioni

void ISRgest(void);					// I.S.R.
void init_sys(void);				// system init					
void EEPROMWRITE(char, char);
char EEPROMREAD(char);
