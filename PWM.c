/*******************************************************
Name: PWM.c
Date: 9/27/2013
Created By: Don Truex
Revision History:
MG 10/10/2013 Modified and commented
MG 10/30/2013 Corrected error in PTPER formula
 
Description:
This file initializes the PWM 1H on the dsPIC,
sets the PWM frequency, and sets the initial output to 0.
In order to set the PWM frequency, define PTPER = 118e6*8/(f*p)
where f is freq in Hz and p is a prescalar (see below), where
p is selected such that PTPER is a large 16-bit word
(e.g., between 10k and 20k). Strictly speaking PTPER must be between
0x0010 and 0xFFF8. In order to output a PWM waveform with duty cycle d,
you can calculate the output using:

PDC1 = ((PTPER+8)*d)/100;

where PDC1 is a 16-bit pulse width register (defined by the compiler)
and the duty cycle d should be defined as a 16-bit unsigned integer:
UINT16 d ; // 0<=d<=100
 
In order to calculate the number to load into the pulse width
register, you need to type cast into a temporary 32-bit unsigned
integer (so the product can exceed 64k), then saturate between
0 and the PTPER (0 and 100% DC), then load into the PDC7 register.
Example code:
 
      PDC1_temp = ((UINT32)(PTPER+8)*(INT32)d)/100 ;
      if (PDC1_temp < 0) PDC1_temp = 0;
      if (PDC1_temp > PTPER+8) PDC1_temp = PTPER+8;
      PDC1 = (UINT16)PDC1_temp ; // load DC into register

References:
See section 43 High-speed PWM datasheet (document no. 70323E-1)

*********************************************************/
//#include "p33fxxxx.h"
//#include "Externals.h"
#include "Defs.h"
#include "Includes.h"
#include "Externals.h"
void PWMInit(void);

void PWMInit(void)
{
	// Setup for the PWM clock to use the Primary Oscillator as the REFCLK */
	//((OSC * 16) / APSTSCLR) = (7.37 * 16) / 1 = 117.92 MHz ~ 118 MHz */
//	ACLKCONbits.ASRCSEL = 1; // Primary Oscillator is the Clock Source */
//	ACLKCONbits.FRCSEL = 0; // Input clock source is determined by ASRCSEL bit setting */
//	ACLKCONbits.SELACLK = 1; // Auxiliary Oscillator provides the clock source */
//	ACLKCONbits.APSTSCLR = 7; // Divide Auxiliary clock by 1 */
//	ACLKCONbits.ENAPLL = 1; // Enable Auxiliary PLL */
	ACLKCON = 0xe780;
	while(ACLKCONbits.APLLCK != 1); // Wait for Auxiliary PLL to Lock */
        /**************************/
      	/* Set PWM frequency here */
        /**************************/
        // Equation 43.4 in DS70323E where aux clock is 118e6 Hz
        // PTPER = 118e6*8/(f*p) - 8 ;
        // where p is prescalar and f is PWM freq in Hz
        // Note that PTPER is a 16-bit word (0x0010<=PTPER<=0xFFF8)
        // which is 16<=PTPER<=65528
        // Note: Lowest PWM frequency at PTPER = 0xFFF8 (65528)
        // f(lowest) ~= 118e6*8/(65528*64) = 244 Hz

        // Use f=1kHz for the LED brightness
        // For f=1kHz and p=64, PTPER=(118e6)(8)/((1000)(64))-8 = 15992

        /* Select PWM prescaler (see page 235 of datasheet) */
        // Prescalar p=1
	// PTCON2bits.PCLKDIV = 0;
        // Prescalar p=2
	// PTCON2bits.PCLKDIV = 1;
	// Prescalar p=4 
	// PTCON2bits.PCLKDIV = 2;
	// Prescalar p=8 
	// PTCON2bits.PCLKDIV = 3;
        // Prescalar p=16 
	// PTCON2bits.PCLKDIV = 4;
        // Prescalar p=32 
	// PTCON2bits.PCLKDIV = 5;
        // Prescalar p=64 
	PTCON2bits.PCLKDIV = 6;

        #define PWMPERIOD 2942 //5kHz
	PTPER = PWMPERIOD; //Primary period
	//setup PWM 1H 
	PWMCON6 = 0x0000;
	IOCON6bits.PENH = 1;  //enable PWM H output
	IOCON6bits.PENL = 1;  //enable PWM L output.
	IOCON6bits.PMOD = 0b11; //hi low are independent
	LEBDLY6bits.LEB = 0;
	DTR6 = 0; //
    // --- PWM1: enable L pin and run it independent of H ---
    IOCON6bits.POLH = 0;         // active-high
    IOCON6bits.POLL = 0;         // active-high       


    /* Set the duty cycle of the PWM output here */
	PDC6 = 0; // Initialize to 0% duty cycle
    SDC6 = 0;                    // duty register for the L pin
    
    // Make sure old channels are off
    PWMCON1 = 0;
    IOCON1bits.PENH = 0;
    IOCON1bits.PENL = 0;
    
    PWMCON2 = 0;
    IOCON2bits.PENH = 0;
    IOCON2bits.PENL = 0;


	PTCONbits.PTEN = 1; // Uncomment to enable output

    
}
/*Registered to pin 67*/
void writePWM6H(UINT16 DC)
{
    UINT32 temp = ((UINT32)(PTPER + 8) * DC) / 100;
    if (temp > (UINT32)(PTPER + 8))
        temp = (UINT32)(PTPER + 8);
    PDC6 = (UINT16)temp;
}

/*Registered to pin 66*/
void writePWM6L(UINT16 DC)
{
    UINT32 temp = ((UINT32)(PTPER + 8) * DC) / 100;
    if (temp > (UINT32)(PTPER + 8))
        temp = (UINT32)(PTPER + 8);
    SDC6 = (UINT16)temp;
}
