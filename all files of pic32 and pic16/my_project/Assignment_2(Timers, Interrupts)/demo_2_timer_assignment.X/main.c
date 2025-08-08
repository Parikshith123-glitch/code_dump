
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
#pragma config FPLLIDIV = DIV_1         // System PLL Input Divider (2x Divider) -- 1
#pragma config FPLLRNG = RANGE_8_16_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_40        // System PLL Multiplier (PLL Multiply by 94) -- 47
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (2x Divider) -- 2
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
#pragma config JTAGEN = OFF              // JTAG Enable (JTAG Port Enabled)
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

#include <xc.h>
#include <p32xxxx.h>

// Function Declaration

void timer_1_initialization(void);
void LED_blink_for_timer_1(void);

void timer_2_3_initialization(void);
void LED_blink_for_timer_2_3(void);

// This has 16 and 32 bit timer code.

int main()
{
    char flag  = 1;                  // Flag variable is used to switch between the timers.
    
    TRISCbits.TRISC14 = 0;           // Setting the pin as output using TRISx
    TRISCbits.TRISC13 = 0;

    timer_1_initialization();        // Function call for timer initializations
    timer_2_3_initialization();
    
    while(1)
    {
        if(flag == 1){                   // Flag variable to switch between the 16 and 32 bit timers.
            
            LED_blink_for_timer_1();     // Actual LED blinking for timer 1.
            flag = 0;                    
            
        } else {
            
            LED_blink_for_timer_2_3();   // Actual LED blinking for timer 2 and timer 3. 
            flag = 1;
        
        }
    }
    return 0;
}

/********************************************************************************
 * FUNCTION NAME    : timer_1_initialization
 * DESCRIPTION      : This function initializes the timer 1 with the required 
 *                         values of timer register and period timer.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void timer_1_initialization(void)
{
    
    T1CONbits.ON = 0;           // Timer OFF
    T1CONbits.TCS = 0;          // PBCLK Clock source
    T1CONbits.TCKPS = 3;        // Pre-scaler 
    
    TMR1 = 0;                   // resetting the timer register
    PR1 = 0xFFFF;               // setting the period register 
    
    T1CONbits.ON = 1;           // Switching the Timer ON
}

/********************************************************************************
 * FUNCTION NAME    : LED_blink_for_timer_1
 * DESCRIPTION      : This function is for LED blinking for timer 1
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void LED_blink_for_timer_1(void)
{
    
    LATCbits.LATC13 = 1;        // Switching the LED ON
       
    while(TMR1 < PR1);          // while delay
    
    TMR1 = 0;                   // Resetting the timer register
    
    LATCbits.LATC13 = 0;        // Switching the LED OFF
    
    while(TMR1 < PR1);          // while delay

    TMR1 = 0;                   // Resetting the timer register
}

/********************************************************************************
 * FUNCTION NAME    : timer_2_3_initialization
 * DESCRIPTION      : This function initializes the timer 2 and timer 3 as a 32 bit timer 
 *                      with the required values of timer register and period register.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void timer_2_3_initialization(void)
{
    T2CONbits.ON = 0;           // Timer 2 & 3 are switched OFF
    T3CONbits.ON = 0;       
    
    T2CONbits.T32 = 1;          // This bit is required for turning the timers to 32-bit timer
    T2CONbits.TCS = 0;          // PBCLK Clock source
    T2CONbits.TCKPS = 2;        // Pre-scaler 
    
    TMR2 = 0;                   // resetting the timer registers
                         
    PR2 = 0x00FFFFFF;           // setting the period register 
  
    T2CONbits.ON = 1;           // Switching the Timer ON
}

/********************************************************************************
 * FUNCTION NAME    : LED_blink_for_timer_2_3
 * DESCRIPTION      : This function is for LED blinking for timer 2 and timer 3
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void LED_blink_for_timer_2_3(void)
{    
    LATCbits.LATC14 = 1;        // switching the LED ON
       
    while(TMR2 < PR2);          // while delay
    
    TMR2 = 0;                   // Resetting the timer register after the count
    TMR3 = 0;
    
    LATCbits.LATC14 = 0;        // switching the LED OFF
    
    while(TMR2 < PR2);          // while delay
    
    TMR2 = 0;                   // resetting the timer register after the count.
    TMR3 = 0;
}