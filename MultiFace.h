/*
 *	MultiFace header file
 *	
 */

// prototipi funzioni

void ISRgest(void);					// I.S.R.
void ServoCiclo(void);				// Gestione servi
void ADCCiclo(void);				// Gestione ADC
void GPIOCiclo(void);				// Gestione GPIO
void PWMCiclo(void);				// Gestione PWM
void Parser(void);					// Parser
void SRFping(unsigned char);		// Ping Sonar
void SRFread(unsigned char);		// legge distanza
void CMPS03(unsigned char);			// legge bussola
void MD22(char, char, char);		// MD22
void MD22mod (char, char);			// modo MD22
void GPIO_read(void);				// GPIO
void init_sys(void);				// init sistema					
void EEPROMWRITE(char, char);		// scrive EEPROM
char EEPROMREAD(char);				// legge EEPROM
void SRF_Ad_Ch(char, char);			// Address change sonar
void CPMS03_Ad_Ch(char, char);		// Address change compass
void I2CW(void);					// I2C generic write
void I2CR(void);					// I2C generic read
