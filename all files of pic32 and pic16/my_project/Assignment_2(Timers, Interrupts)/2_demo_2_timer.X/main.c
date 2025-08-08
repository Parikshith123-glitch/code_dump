
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
#pragma config FPLLIDIV = DIV_1         // System PLL Input Divider (1x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_40        // System PLL Multiplier (PLL Multiply by 40)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (2x Divider)
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
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
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

/*******************************************************************************
 System header files
 *******************************************************************************/

#include <xc.h>         // PIC controller definitions
#include <p32xxxx.h>    // PIC32MZ2048EFH064 library header file 

/*Function Prototype*/

void sixteen_bit_sync_operation_init(void);
void sixteen_bit_sync_external_operation_init(void);

void three_two_bit_sync_timer_init(void);
void three_two_bit_extenal_operation_init(void);

void timer_1_start(void);
void timer_2_start(void);
void timer_4_5_start(void);
void timer_6_7_start(void);

int main(int argc, char *argv[])
{
    /*Function calling*/

    sixteen_bit_sync_operation_init();
    sixteen_bit_sync_external_operation_init();
    
    three_two_bit_sync_timer_init();
    three_two_bit_external_operation_init();
    
    while(1)
    {
        timer_1_start();
        timer_2_start();
        timer_4_5_start();
        timer_6_7_start();
    }
    return 0;
}

/********************************************************************************
 * FUNCTION NAME    : sixteen_bit_sync_operation
 * DESCRIPTION      : This function initializes and runs timer 1 synchronously
 *                      using PBCLK clock.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void sixteen_bit_sync_operation_init(void)
{
    
    T1CONbits.ON = 0;           // Switching OFF the timer 1
    T1CONbits.TCS = 0;          // Setting the PBCLK clock source bit
    T1CONbits.TCKPS = 3;        // Setting the pre-scaler
    
    TMR1 = 0;                   // Re-setting the Timer register to zero
    PR1 = 0xFFFF;               // Setting the max value for Period Register
      
    T1CONbits.ON = 1;           // Switching ON the Timer

}

/********************************************************************************
 * FUNCTION NAME    : timer_1_start
 * DESCRIPTION      : This function runs the timer 1 infinitely.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void timer_1_start(void)
{
    while(TMR1 < PR1);          // While delay
    
    TMR1 = 0;                   // Re-setting the Timer register to zero after the while delay
}

/********************************************************************************
 * FUNCTION NAME    : sixteen_bit_sync_external_operation
 * DESCRIPTION      : This function initializes and runs timer 2 synchronously
 *                      using External clock source.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void sixteen_bit_sync_external_operation_init(void)
{    
    T2CONbits.ON = 0;           // Switching OFF the timer 2
    T2CONbits.TCS = 1;          // Setting the External clock source bit
    T2CONbits.TCKPS = 3;        // Setting the pre-scaler 
    
    TMR2 = 0;                   // Re-setting the Timer register to zero
    PR2 = 0xFFFF;               // Setting the max value for Period Register
    
    T2CONbits.ON = 1;           // Switching ON the Timer

}

/********************************************************************************
 * FUNCTION NAME    : timer_2_start
 * DESCRIPTION      : This function runs the timer 1 infinitely.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void timer_2_start(void)
{
    while(TMR2 < PR2);          // Wait until the TMRx is equal to PRx
    
    TMR2 = 0;                   // Re-setting the Timer register to zero after the while delay
}

/********************************************************************************
 * FUNCTION NAME    : three_two_bit_sync_timer
 * DESCRIPTION      : This function initializes and runs timer 4 and 5 as a
 *                      32-bit timer synchronously using PBCLK clock.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void three_two_bit_sync_timer_init(void)
{
    
    T4CONbits.ON = 0;           // Setting the timer 4 as OFF
    T5CONbits.ON = 0;           // Setting the timer 5 as OFF
    T4CONbits.TCS = 0;          // Setting the clock source as PBCLK clock
    
    T4CONbits.T32 = 1;          // By setting this bit, this becomes a 32 bit timer 
    T4CONbits.TCKPS = 5;        // Setting the pre-scaler
    
    TMR4 = 0;                   // Setting  the timer register as 0
    
    PR4 = 0x00FFFFFF;           // Setting this to the max value using 2 16 bit timers.
    
    T4CONbits.ON = 1;           // Switch ON the timer
     
}

/********************************************************************************
 * FUNCTION NAME    : timer_4_5_start
 * DESCRIPTION      : This function runs the timer 4 and 5 as a single 32 bit timer infinitely.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void timer_4_5_start(void)
{
    while(TMR4 < PR4);          // Wait until the TMRx is equal to PRx
    
    TMR4 = 0;                   // Setting  the timer register as 0 after the while delay
}

/********************************************************************************
 * FUNCTION NAME    : three_two_bit_external_operation
 * DESCRIPTION      : This function initializes and runs timer 4 and 5 as a
 *                      32-bit timer synchronously using PBCLK clock.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void three_two_bit_external_operation_init()
{
    T6CONbits.ON = 0;           // Setting the timer 6 as OFF
    T7CONbits.ON = 0;           // Setting the timer 7 as OFF
    T6CONbits.TCS = 1;          // Setting the clock source as external clock source
    
    T6CONbits.T32 = 1;          // Setting this bit make the two timers as a single 32 bit timer
    T6CONbits.TCKPS = 5;        // Pre-scaler setting 
    
    TMR6 = 0;                   // Timer register resetting
    PR6 = 0x000FFFFF;           // Setting the Period register
    
    T6CONbits.ON = 1;           // Switching ON the 32 bit timer 
}

/********************************************************************************
 * FUNCTION NAME    : timer_6_7_start
 * DESCRIPTION      : This function runs the timer 6 and 7 as a single 32 bit timer infinitely.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void timer_6_7_start(void)
{
    while(TMR6 < PR6);          // While delay
    
    TMR6 = 0;                   // Timer register resetting after the while delay
}

