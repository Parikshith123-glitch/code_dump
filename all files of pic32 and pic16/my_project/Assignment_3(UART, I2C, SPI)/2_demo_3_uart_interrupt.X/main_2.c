
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
#include <sys/attribs.h>            // This header file is required for writing an ISR

// Function Prototypes

void uart_interrupt_init(void);
void uart_receive(void);

char c = 0;

int main()
{    
    // Function Call
    
    uart_interrupt_init();
    
    while(1)
    {
        
    }
    return 0;
}

/***********************************************************************************
 * FUNCTION NAME    : uart_interrupt_init
 * DESCRIPTION      : This function configures all the necessary bits and registers
 *                      in interrupt mode for the transmission of data through UART1.
 * ARGUMENTS        : None
 * RETURNS          : void
 **********************************************************************************/

void uart_interrupt_init(void)
{
    
    asm volatile("di");             // Disable all interrupts
    asm volatile("ehb");            // Execution Hazard Barrier
    
    INTCON  = 0;                    // Clears the INTCON register
    
    U1MODEbits.ON = 0;              // Setting the UART OFF for configuration.
    U1MODEbits.STSEL = 0;           // Selection of number of stop bits in UART frame.
    U1MODEbits.PDSEL = 0;           // Selection of parity and number of data bits in UART frame.
    U1MODEbits.BRGH = 0;            // Setting the standard 16x baud rate generation.

    U1STAbits.URXEN = 1;            // Receive enable bit in status register.
    U1STAbits.UTXEN  = 1;           // Transmit enable bit in status register.
    
    RPC13R = 1;                     // As this is PPS, we are setting the Port C Pin 13 as UART Transmit
    U1RXR = 7;                      // We are receiving on Port C - 14 pin.
    
    U1MODEbits.ON = 1;              // switching ON UART 1.

    U1BRG = 611;                    // Setting the BRG(Baud Rate Generation) value according to 
                                    // system frequency to get proper baud rate(bits/sec) SYSCLK = 188 MHz
    
    U1STAbits.URXISEL = 0;          // Generate an interrupt even whenever a single character is received.
    
    IFS3bits.U1RXIF = 0;            // Clear UART RX interrupt flag
    IEC3bits.U1RXIE = 1;            // Enable UART RX interrupt enable
    
    IPC28bits.U1RXIP = 3;           // Setting the priority
    IPC28bits.U1RXIS = 0;           // Setting the sub-priority

    asm volatile("ei");             // enable all interrupts
     
}

/***********************************************************************************
 * FUNCTION NAME    : UART_receive
 * DESCRIPTION      : This function is configured to receive data from transmit
 * ARGUMENTS        : None
 * RETURNS          : void
 **********************************************************************************/

void uart_receive(void)
{
    char ch1 = 0;
    while(IFS3bits.U1RXIF == 0);
    IFS3bits.U1RXIF = 0;                // Resetting the Interrupt flag in the ISR

    
    ch1 = U1RXREG;                      // Read received character
    //while (U1STAbits.UTXBF == 1);     // if transmit buffer is full, so wait
    
    U1TXREG = ch1;                      // send back the modified character
    
    while(U1STAbits.TRMT == 0);         // while this is 0, there is still data to send empty it fully and then it changes to 1
                                        // Once this changes to 1, that means there is no data in the FIFO or the shift register.    
}

/********************************************************************************
 * FUNCTION NAME    : UART_ISR_handler
 * DESCRIPTION      : This ISR is when even a single char is sent through UART
 *                      an interrupt is generated.
 * ARGUMENTS        : None
 * RETURNS          : void
 *******************************************************************************/

void __ISR(_UART1_RX_VECTOR, IPL6AUTO) UART_ISR_handler(void)
{
    char ch;
    
    if (IFS3bits.U1RXIF)            // Check if interrupt is due to RX
    {
        ch = U1RXREG;               // Read received character
        while (U1STAbits.UTXBF);    // Wait if TX buffer is full
        U1TXREG = ch;               // Transmit the same character (echo)

        IFS3bits.U1RXIF = 0;        // Clear RX interrupt flag
    }
}


/* Transmit interrupt start */

char message[] = "Hello UART\n";
int index = 0;

void __ISR(_UART1_TX_VECTOR, IPL6AUTO) UART1_TX_ISR(void)
{
    if (IFS3bits.U1TXIF) {
        if (message[index] != '\0') {
            U1TXREG = message[index++];
        } else {
            IEC3bits.U1TXIE = 0;  // Disable TX interrupt after sending
            index = 0;
        }
        IFS3bits.U1TXIF = 0;      // Clear TX interrupt flag
    }
}

/* Transmit interrupt end*/
