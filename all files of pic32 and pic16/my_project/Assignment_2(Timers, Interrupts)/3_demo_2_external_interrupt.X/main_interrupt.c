
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
#pragma config FPLLMULT = MUL_100       // System PLL Multiplier (PLL Multiply by 100)
#pragma config FPLLODIV = DIV_8         // System PLL Output Clock Divider (8x Divider)
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
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = ON              // JTAG Enable (JTAG Port Enabled)
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
#include <sys/attribs.h>

// Function Prototype

void interrupt_init(void);

int main()
{
    TRISCbits.TRISC13 = 0;     // Setting PORTC 13th bit as output
   
    TRISDbits.TRISD0 = 1;      // This pin is of INT0 (External Interrupt 0)
    
    interrupt_init();          // Function calling
    
    while(1)
    {
        
    }
    
    return 0;
}

/********************************************************************************
 * FUNCTION NAME    : interrupt_init
 * DESCRIPTION      : This function definition is the initialization of external 
 *                      interrupt 0.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void interrupt_init(void)
{
 
    asm volatile("di");        // Disable Interrupts - Disables all interrupts
    asm volatile("ehb");       // Execution Hazard Barrier
    
    INTCON = 0;                // Setting the Interrupt Control Pin as 0
    
    INTCONbits.INT0EP = 0;     // 1 = Rising Edge, 0 = Falling Edge
    
    IFS0bits.INT0IF = 0;       // Resetting interrupt flag status
    
    IPC0bits.INT0IP = 6;       // Setting Priority 
    IPC0bits.INT0IS = 0;       // Setting Sub-Priority 
    
    asm volatile("ei");        // Enable Interrupts - Re-enables all interrupts
    
    IEC0bits.INT0IE = 1;       // Enable Interrupt 0

}

/********************************************************************************
 * FUNCTION NAME    : External_0_ISR_handler
 * DESCRIPTION      : This function is the ISR for INT0. Whenever an external 
 *                      interrupt occurs on INT0 Pin(Pin 46) this ISR executes. 
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void __ISR(_EXTERNAL_0_VECTOR, IPL6AUTO)External_0_ISR_handler(void)
{
    LATCbits.LATC13 ^= 1;      // Toggling the LED to indicate ISR is triggered
    
    IFS0bits.INT0IF = 0;       // Resetting the flag status
}