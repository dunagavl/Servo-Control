/*******************************************************
Name: Main.c
Date: 10/3/2025
Created By: DT/MG/AH
Description: Main source code file for dsPIC breakout board test.
Now implements a discrete-time PD servo using Tustin approximation.
*******************************************************/

#include "Defs.h"
#include "Includes.h"
#include "Globals.h"

// ===== External function prototypes (from other files) =====
extern void InitOsc(void);      // Located in Init.c
extern void InitGPIO(void);     // Located in Init.c
extern void InitGlobs(void);    // Located in Init.c
extern void InitEcan1(void);    // Located in Init.c
extern void TimersInit(void);   // Located in Timers.c
extern void ADCInit(void);
extern void PWMInit(void);
extern void writePWM6L(UINT16 DC);
extern void writePWM6H(UINT16 DC);

extern void Ecan1Init(void);    // Located in ECAN1Config.c
extern void CanSend(UINT16 ID);
extern void CanSendAll(void);
extern void CanSend0x20(void);
extern void CanSend0x21(void);
extern int  CanParse(void);

extern UINT8 USBUartConfig(void);
extern UINT8 USBUartParse(void);
extern void  USBUartTxAll(void);

// QEI/IMU (encoder) stuff
extern void initQEI(void);

// ===== Encoder constants: PEC12R = 24 pulses/rev, x4 = 96 counts/rev =====
#define ENCODER_PPR            1000u
#define QEI_COUNTS_PER_REV     (ENCODER_PPR * 4u)   // 4000

// ===== PD controller (Tustin) coefficients =====
// From MATLAB: c2d(..., 'tustin') with kp=4, kd=0.2, fc=25 Hz, Ts=0.001 s
#define A1        0.8544f     // u[k-1] coefficient
#define B0       33.13f       // e[k] coefficient
#define B1      -32.55f       // e[k-1] coefficient (note minus sign)

// ===== Physical limits and conversion constants =====
#define V_SUPPLY  24            // Treat ±24 V as full command
#define TWO_PI    6.28318530718f
#define PI        3.14159265359f

float e_prev = 0.0f;
float u_prev = 0.0f;

// ===== Helpers: conversions and PD step =====

// Convert encoder counts [0..QEI_COUNTS_PER_REV-1] to radians [0..2?)
static inline float encoderCountsToRadians(INT16 counts)
{
    // POS1CNT runs 0..QEI_COUNTS_PER_REV-1, no need for wrapping here.
    return (TWO_PI * (float)counts) / (float)QEI_COUNTS_PER_REV;
}

// Degrees to radians
static inline float degToRad(float deg)
{
    return deg * (PI / 180.0f);
}

// One step of the discrete-time PD controller
// theta_ref_rad: desired angle [rad]
// theta_meas_rad: measured angle [rad]
static float PD_step(float theta_ref_rad, float theta_meas_rad)
{

    float e = theta_ref_rad - theta_meas_rad;   // error in radians

    // u[k] = A1*u[k-1] + B0*e[k] + B1*e[k-1]
    float u = A1 * u_prev + B0 * e + B1 * e_prev;

    // Saturate to ±V_SUPPLY
    if (u >  V_SUPPLY) u =  V_SUPPLY;
    if (u < -V_SUPPLY) u = -V_SUPPLY;

    e_prev = e;
    u_prev = u;

    return u;   // volts
}

// Map controller output voltage to H-bridge direction + PWM duty.
// Uses PWM6H for positive voltage (forward) and PWM6L for negative (reverse).
static void setMotorFromVoltage(float u)
{

    // Convert to duty percentage [0..100]
    INT16 duty = (INT16)(u *100 / V_SUPPLY);
    if (duty < -100)   duty = -100;
    if (duty > 100) duty = 100;
    
    if (duty > 5)
    {
        // Forward direction: H side PWM, L side off
        LATDbits.LATD7 = 0;    // disable L side Pin 69
        LATDbits.LATD6 = 1;    // enable H side Pin 68
        writePWM6H(duty);      // Pin 67
        writePWM6L(0);         // Pin 66
    }
    else if (duty < -5)
    {
        // Reverse direction: L side PWM, H side off
        LATDbits.LATD6 = 0;
        LATDbits.LATD7 = 1;
        writePWM6H(0);
        writePWM6L(abs(duty));
    } else 
    {
        // Turn off the H-Bridge
        LATDbits.LATD6 = 0;
        LATDbits.LATD7 = 0;
        writePWM6H(0);
        writePWM6L(0);
    }    
}

/*******************************************************
 * main()
 *******************************************************/
int main(void)
{
    // Locals
    INT16   dataIn[4] = {0, 0, 0, 0};

    // ===== System init =====
    InitOsc();
    InitGPIO();
    InitGlobs();
    TimersInit();
    Ecan1Init();
    ADCInit();
    PWMInit();
    USBUartConfig();
    initQEI();                // start QEI

    ALLREDLEDSOFF;

    // H-bridge GPIO: RD6, RD7 as outputs, initially low
    TRISDbits.TRISD6 = 0;     // pin 68
    TRISDbits.TRISD7 = 0;     // pin 69
    LATDbits.LATD6 = 0;
    LATDbits.LATD7 = 0;

    // Global interrupt config
    INTCON1bits.NSTDIS = 0;   // allow nested interrupts
    SRbits.IPL = 0;           // CPU priority level

    // ===== Main loop (1 kHz control via gTimers[ONE_MS]) =====
    while (1)
    {
        // 1 kHz "tick" driven by TimersInit / ISR updating gTimers[ONE_MS]
        if (gTimers[ONE_MS] == 0)
        {
            gTimers[ONE_MS] = 1;   // reload 1 ms timer

            // ---- USB in (get reference angle, etc.) ----
            USBUartParse();
            dataIn[0] = gUSBdataIn[0];
            dataIn[1] = gUSBdataIn[1];
            dataIn[2] = gUSBdataIn[2];
            dataIn[3] = gUSBdataIn[3];

            // Interpret dataIn[0] as desired angle in degrees (0..360)
            float theta_ref_deg = (float)dataIn[0];
            float theta_ref     = degToRad(theta_ref_deg);

            // ---- Read encoder and convert to radians ----
            INT16 counts      = (INT16)POS1CNT;          // 0..QEI_COUNTS_PER_REV-1
            float theta_meas  = encoderCountsToRadians(counts);
            float theta_meas_deg = theta_meas * (180.0f / PI);

            // ---- PD control step ----
            float voltage = PD_step(theta_ref, theta_meas);    // volts

            // ---- Drive motor via H-bridge ----
            setMotorFromVoltage(voltage);

            // ---- USB out (stream at 1 kHz) ----
            // You can plot these in MATLAB for response analysis.
            gUSBdataOut[0] = 0; //(INT16)counts;              // raw encoder counts
            gUSBdataOut[1] = (INT16)theta_ref_deg;       // command angle [deg]
            gUSBdataOut[2] = (INT16)theta_meas_deg;      // measured angle [deg]
            gUSBdataOut[3] = 0; //(INT16)(voltage * 1000.0f);       // control voltage [mV]

            if (gSendUSB == 1)
            {
                USBUartTxAll();
            }
        }
    }
}

/*******************************************************
 * QEI init (unchanged from your previous code)
 *******************************************************/
void initQEI(void)
{
    ADPCFG |= 0x0038;
    QEI1CONbits.QEIM = 0;
    QEI1CONbits.CNTERR = 0;
    QEI1CONbits.QEISIDL = 0;
    QEI1CONbits.SWPAB = 0;
    QEI1CONbits.PCDOUT = 0;
    DFLT1CONbits.CEID = 1;
    DFLT1CONbits.QEOUT = 1;
    DFLT1CONbits.QECK = 3;

    TRISDbits.TRISD11 = 1;
    TRISDbits.TRISD0  = 1;

    POS1CNT = 0;

    QEI1CONbits.QEIM = 6;              // x4 quadrature, reset on MAX1CNT

    return;
}
