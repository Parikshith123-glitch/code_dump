
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
#pragma config FPLLMULT = MUL_47        // System PLL Multiplier (PLL Multiply by 47)
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

// Function Prototypes

void ADC_init(void);
uint16_t ADC_check(void);

int main()
{
    uint16_t output;
    ADC_init();                 // function calling 
    
    while(1)
    {
        output = ADC_check();   // catching the output in a variable and observing 
                                // the output through this variable
    }
    
    return 0;
}

/***********************************************************************************
 * FUNCTION NAME    : ADC_init
 * DESCRIPTION      : This function configures all the necessary bits and registers
 *                      for the proper ADC working.
 * ARGUMENTS        : None
 * RETURNS          : void
 **********************************************************************************/

void ADC_init(void)
{
    ADC0CFG = DEVADC0;              // copying the config to that particular ADC
    ADCCON1bits.ON = 0;             // turning ON the ADC
    
    ADCCON3bits.ADCSEL = 3;         // input clock source 
    ADCCON3bits.CONCLKDIV = 1;      // control clock => Tq = 2 * Tclk [000001]
    ADCCON3bits.VREFSEL = 0;        // voltage selection reference bits
    
    ADCCON1bits.STRGSRC = 1;        //  00001 = Global software trigger (GSWTRG) is self-cleared on the next clock cycle
    
    ADC0TIMEbits.SELRES = 3;        // Setting the resolution
    ADC0TIMEbits.ADCDIV = 2;        // Tad = 2 * Tq [T divides the Tclk so this is used by ADC for conversion] - ADC's conversion speed, 
                                    // large division means slower clock and thus a slower conversion.
    ADC0TIMEbits.SAMC = 5;          // SAMC = 5 * Tad [This should be set bit high so that there is time for sampling by the ADC]
    
    ADCTRGMODEbits.SH0ALT = 0;      // Analog input selection bit = AN0
    
    ADCIMCON1bits.DIFF0 = 0;        // Single ended mode [difference with AN0 voltage and the ground]
    ADCIMCON1bits.SIGN0 = 0;        // For unsigned value 
    
    ADCCON1bits.ON = 1;
    
}

/***********************************************************************************
 * FUNCTION NAME    : ADC_check
 * DESCRIPTION      : This function takes the data from the data register and returns
 *                      the output to the main function.
 * ARGUMENTS        : None
 * RETURNS          : uint16_t
 **********************************************************************************/

uint16_t ADC_check(void)
{
    uint16_t result;
    
    while(!ADCCON2bits.BGVRRDY);    // Wait until the reference voltage is ready
    while(ADCCON2bits.REFFLT);      // Wait if there is a fault with the reference voltage

    ADCANCONbits.ANEN0 = 1;         // ADC analog warm up control register
    while(!ADCANCONbits.WKRDY0);    // Wait until the ADC0 is ready
    
    ADCCON3bits.DIGEN0 = 1;         // enabling ADC0
    
    ADCCON3bits.GSWTRG = 1;             // Start the conversion when global trigger is set to 1.
    while (ADCDSTAT1bits.ARDY0 == 0);   // Wait the conversions to complete
    result = ADCDATA0;                  // copy the data to the result variable
    
    return result;
}

    
