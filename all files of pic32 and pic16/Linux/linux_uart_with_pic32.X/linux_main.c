
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
#pragma config FPLLIDIV = DIV_4         // System PLL Input Divider (4x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_50        // System PLL Multiplier (PLL Multiply by 10)
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

#include <xc.h>         // PIC Controller Definitions    
#include <p32xxxx.h>    // PICMZ2048EFH064 library header file   

// Function Declarations

void UART_init(void);
void UART_Receive(void);
void delay_ms(unsigned int ms);

volatile char rec_value[50];
volatile int i = 0, j = 0;


int main()
{ 
    TRISCbits.TRISC14 = 1;     // RC14 as input
    TRISEbits.TRISE1 = 0;      // LED output
    char ch=0;
    UART_init();    

    while(1)
    {       
         while (IFS3bits.U1RXIF == 0);       // no new data yet in receive buffer, wait for data
            IFS3bits.U1RXIF=0;          // If this becomes 1, there is data to read.
            U1STAbits.OERR=0;
                     
    ch = U1RXREG;                       // Read received character and assign it to ch.
        
   if(ch==2)
      LATEbits.LATE1 = 0;
   else
     LATEbits.LATE1 = 1;  
   
ch=0;    
        //for(j = 0; j < 10; j++);
//        if (U1STAbits.URXDA) {
//            char ch = U1RXREG;
//            for(j = 0; j < 10; j++);
//            if(ch==0xAA)
//            LATEbits.LATE1 = 0;
//            delay_ms(5000);
//
//            // Optional: store received
//            rec_value[i++] = ch;
//            if (i >= 50) i = 0;
//        } else {
//            for(j = 0; j<100; j++){
////            LATEbits.LATE1 = 0;
////            delay_ms(1000);
//            LATEbits.LATE1 = 1;
//            delay_ms(1000);
//            }
        //}
    }
    return 0;
}


void UART_init(void)
{
    U1MODEbits.ON = 0;      // UART is switched OFF for configuration 
    
    U1MODEbits.BRGH = 0;    // 16x Standard speed mode 
    U1MODEbits.PDSEL = 0;   // Parity and data selection bits (8, N, 1)
    U1MODEbits.STSEL = 0;   // Selection of number of stop bits = 1
    
    U1STAbits.UTXEN = 1;    // Enabling UART Transmitting bit in status register 
    U1STAbits.URXEN = 1;    // Enabling UART Receiving bit in status register
    
    RPC13R = 1;             // As this is PPS, we are setting the Port C Pin 13 as UART Transmit (TX) Port 13 to 5th Port in TTB
    U1RXR = 7;              // As this is PPS, we are setting the Port C Pin 14 as UART Receive  (RX) Port 14 to 4th Pin in TTB
    
//    U2RXR = 4; //Pin no 52 - 2nd RX to RPD4 UART 2
//    RPD5R = 2; //Pin no 53 - 2nd TX to RPD5
            
    U1BRG = 32;            // Setting the BRG(Baud Rate Generation) value according to 
                            // system frequency to get proper baud rate(bits/sec) SYSCLK = 188 MHz 
                            // but the PBCLK2 is SYSCLK/2^n so as this is in the 2nd peripheral line PBCLK = 188/2^1 = 94MHz. 
                            // By using this and doing the calculations we get the baud rate as 9600 bits/sec
    
    U1MODEbits.ON = 1;      // UART is switched ON for transmission

}
 
//---------------------------------------------------------------------------------------------------------------------- 


void UART_Receive(void)
{
    int ch, count;

    // Wait until a character is received
    while (!U1STAbits.URXDA);  // Corrected: wait while no data

    // Read the received byte
    ch = U1RXREG;

    // Store in buffer
    rec_value[i++] = ch;
    if (i >= 50) i = 0;
    
    LATEbits.LATE1 = 1;
    delay_ms(5000);
    LATEbits.LATE1 = 0;
    delay_ms(5000);

    
    // Echo it back
//    while (U1STAbits.UTXBF);   // Wait if transmit buffer is full
//    U1TXREG = ch;
//
//    // Wait until transmission complete
//    while (!U1STAbits.TRMT);
}



 //---------------------------------------------------------------------------------------------------------------------- 

void delay_ms(unsigned int ms)
{
    unsigned int i, j;
    for(i = 0; i < ms; i++)
    {
        for(j = 0; j < 1000; j++)  // Approximate 1ms delay
        {
            asm("nop");
        }
    }
}