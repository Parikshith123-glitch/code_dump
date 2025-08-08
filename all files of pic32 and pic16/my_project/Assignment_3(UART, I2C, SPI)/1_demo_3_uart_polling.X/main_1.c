
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
#pragma config FPLLIDIV = DIV_1         // System PLL Input Divider (8x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_47        // System PLL Multiplier (PLL Multiply by 80)
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
#pragma config JTAGEN = OFF              // JTAG Enable (JTAG Port Enabled)
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

#include <xc.h>
#include <p32xxxx.h>

// Function Declarations

void UART_init(void);
void UART_Transmit(void);
void UART_Receive(void);

int main()
{   
    // Function calling
    
    UART_init();    
    UART_Transmit();

    while(1)
    {
        UART_Receive();
    }   
    
    return 0;
}

/***********************************************************************************
 * FUNCTION NAME    : UART_init
 * DESCRIPTION      : This function configures all the necessary bits and registers
 *                      for the proper transmission of data through UART1.
 * ARGUMENTS        : None
 * RETURNS          : void
 **********************************************************************************/

void UART_init(void)
{
    U1MODEbits.ON = 0;      // UART is switched OFF for configuration 
    U1MODEbits.BRGH = 0;    // 16x Standard speed mode 
    U1MODEbits.PDSEL = 0;   // Parity and data selection bits (8, N, 1)
    U1MODEbits.STSEL = 0;   // Selection of number of stop bits = 1
    
    U1STAbits.UTXEN = 1;    // Enabling UART Transmitting bit in status register 
    U1STAbits.URXEN = 1;    // Enabling UART Receiving bit in status register
    
    RPC13R = 1;             // As this is PPS, we are setting the Port C Pin 13 as UART Transmit (TX) Port 13 to 5th Port in TTB(Tx of TTb)
    U1RXR = 7;              // As this is PPS, we are setting the Port C Pin 14 as UART Receive  (RX) Port 14 to 4th Pin in TTB(Rx of TTb))

    U1BRG = 611;            // Setting the BRG(Baud Rate Generation) value according to 
                            // system frequency to get proper baud rate(bits/sec) SYSCLK = 188 MHz
                            // 9600 bits/sec
    
    U1MODEbits.ON = 1;      // UART is switched ON for transmission

}

/***********************************************************************************
 * FUNCTION NAME    : UART_Transmit
 * DESCRIPTION      : This function is configured to transmit data to PUTTY
 * ARGUMENTS        : None
 * RETURNS          : void
 **********************************************************************************/

void UART_Transmit(void)
{
    char str[100] = " Hello_SCOPE \r\n";  // Test string to send through UART
    char c;
    unsigned int i = 0,n = 7;           // Loop count variables
        
    while(n > 0)                        // Only execute this 'n' times.
    {  
        while(str[i]!='\0')             // Execute this while condition until '\0' is encountered. 
        {   
            c = str[i++];               // assigning the current character to c and incrementing 
            U1TXREG = c;                // Transmitting the character through UART Transmit register
            
            while(U1STAbits.TRMT == 0); // while this is 0, there is still data to send, empty it fully and then it changes to 1
                                        // Once this changes to 1, that means there is no data in the FIFO or the shift register.
                                        // This bit is READ-ONLY so this is changed by the hardware itself and no need for manual changing.
        }
        
        for(i = 0; i < 90000; i++);     // For delay to avoid over-writing in the buffer 
//        for(i = 0; i < 90000; i++);
//        for(i = 0; i < 90000; i++);

        i = 0;                          // Resetting and decrementing the loop variables
        n--;
    } 
}

/***********************************************************************************
 * FUNCTION NAME    : UART_Receive
 * DESCRIPTION      : This function is configured to receive data from transmit
 * ARGUMENTS        : None
 * RETURNS          : void
 **********************************************************************************/

void UART_Receive(void)
{
    char ch;                            // char variable to re-transmit the variable.
    
    while (IFS3bits.U1RXIF == 0);       // no new data yet in receive buffer, wait for data
            IFS3bits.U1RXIF=0;          // If this becomes 1, there is data to read.
            U1STAbits.OERR=0;
                     
    ch = U1RXREG;                       // Read received character and assign it to ch.

    if(ch >= 'A' && ch <= 'Z')          // We are just modifying the Upper case letters and all the 
                                        // other non-upper case letters/symbols are sent as it is.
    ch = ch + 32;                       

    while (U1STAbits.UTXBF == 1);       //  if transmit buffer is full, so wait
    U1TXREG = ch;                       // Transmit back the variable.
    
    while(U1STAbits.TRMT == 0);         // while this is 0, there is still data to send empty it fully and then it changes to 1
                                        // Once this changes to 1, that means there is no data in the FIFO or the shift register.    
}


