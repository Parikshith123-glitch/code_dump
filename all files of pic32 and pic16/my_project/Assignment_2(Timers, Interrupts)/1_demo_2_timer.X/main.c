
// PIC32MZ2048EFH064 Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config FMIIEN = ON              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = ON              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USBID Selection (Controlled by the USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_1        // System PLL Input Divider (1x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_10     // System PLL Multiplier (PLL Multiply by 100)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (8x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = SPLL             // Oscillator Selection Bits (System PLL)
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disable SOSC)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = ON              // Deadman Timer Enable (Deadman Timer is enabled)

// DEVCFG0
#pragma config DEBUG = ON              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = ON              // JTAG Enable (JTAG Port Enabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config TRCEN = ON               // Trace Enable (Trace features in the CPU are enabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable bit (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = GAIN_2X       // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = GAIN_2X       // Primary Oscillator Gain Control bits (2x gain setting)
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot (Normal EJTAG functionality)

// DEVCP0
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ3
#pragma config TSEQ = 0xFFFF            // Boot Flash True Sequence Number (Enter Hexadecimal value)
#pragma config CSEQ = 0xFFFF            // Boot Flash Complement Sequence Number (Enter Hexadecimal value)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


/*****************************************************************
 System header files
 *****************************************************************/

#include <xc.h>         // PIC controller definitions
#include <p32xxxx.h>    // PIC32MZ2048EFH064 library header file 

// Function Prototypes

void Timer_1_Initialization(void);
void Switching_ON_LED_and_timer_1(void);

void Timer_2_Initialization(void);
void Switching_ON_LED_and_timer_2(void);

int main(int argc, char *argv[]) 
{
    char flag = 0;                  // Flag bit to switch between the timers.
    
    TRISCbits.TRISC13 = 0;          // Setting C port 13th bit as OUTPUT for timer 1.
    TRISCbits.TRISC14 = 0;          // Setting C port 14th bit as OUTPUT for timer 2.
    
    Timer_1_Initialization();       // Function call for timers initialization
    Timer_2_Initialization();           
    
    while(1)
    { 
       // Function calling
        
       Switching_ON_LED_and_timer_1();
             
       Switching_ON_LED_and_timer_2();
    }
    
    return 0;
}


/********************************************************************************
 * FUNCTION NAME    : Timer_1_Initialization
 * DESCRIPTION      : This function initializes the timer 1 with the required 
 *                         values of timer register and period timer.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void Timer_1_Initialization(void)
{     
    T1CONbits.ON = 0;          // Clear the Timer
    
    T1CONbits.TCS = 0;         // Set the clock source as PBCLK (internal)
    T1CONbits.TCKPS = 3;       // Set the pre-scaler as 1 : 256
    
    TMR1 = 0;                  // Clear the Timer register
    PR1 = 0xFFFF;              // Set the Period register to the maximum 
    
    T1CONbits.ON = 1;          // Switch ON the timer.
   
}

/********************************************************************************
 * FUNCTION NAME    : Switching_ON_LED_and_timer_1
 * DESCRIPTION      : This function switches the LED ON at the start of the 
 *                      timer 1 and switches OFF at the end of the timer.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void Switching_ON_LED_and_timer_1(void)
{ 
    LATCbits.LATC13 = 1;        // Switch the LED ON
    while(TMR1 < PR1);          // Wait until the timer is completed 

    LATCbits.LATC13 = 0;        // Switch the LED OFF
    while(TMR1 < PR1);          // Wait until the timer is completed 
    
}

/********************************************************************************
 * FUNCTION NAME    : Timer_2_Initialization
 * DESCRIPTION      : This function initializes the timer 2 with the required 
 *                         values of timer register and period timer.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void Timer_2_Initialization(void)
{        
    T2CONbits.ON = 0;           // Clear the Timer
    
    T2CONbits.TCS = 0;          // Set the clock source as PBCLK (internal)
    T2CONbits.TCKPS = 3;        // Set the pre-scaler as 1 : 256
    
    TMR2 = 0;                   // Clear the Timer register
    PR2 = 0xFFFF;               // Set the Period register to the maximum 
    
    T2CONbits.ON = 1;           // Switch ON the timer.
    
}

/********************************************************************************
 * FUNCTION NAME    : Switching_ON_LED_and_timer_2
 * DESCRIPTION      : This function switches the LED ON at the start of the 
 *                      timer 1 and switches OFF at the end of the timer.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void Switching_ON_LED_and_timer_2(void)
{    
    LATCbits.LATC14 = 1;       // Switch the LED ON
    
    while(TMR2 < PR2);         // Wait until the timer is completed 
     while(TMR2 < PR2);         // Wait until the timer is completed 
      while(TMR2 < PR2);         // Wait until the timer is completed 
 
    LATCbits.LATC14 = 0;       // Switch the LED OFF
    
    while(TMR2 < PR2);         // Wait until the timer is completed 
     while(TMR2 < PR2);         // Wait until the timer is completed 
      while(TMR2 < PR2);         // Wait until the timer is completed 

}