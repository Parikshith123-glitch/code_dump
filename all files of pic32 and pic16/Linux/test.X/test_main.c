#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/attribs.h>
#include <proc/p32mz2048efh064.h>

// Configuration bits for PIC32MZ2048EFH064
#pragma config FMIIEN = ON          // Ethernet RMII/MII Enable
#pragma config FETHIO = ON          // Ethernet I/O Pin Select
#pragma config PGL1WAY = ON         // Permission Group Lock One Way Configuration
#pragma config PMDL1WAY = ON        // Peripheral Module Disable Configuration
#pragma config IOL1WAY = ON         // Peripheral Pin Select Configuration
#pragma config FUSBIDIO = ON        // USB USBID Selection

// DEVCFG2
#pragma config FPLLIDIV = DIV_3     // System PLL Input Divider (3x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ // System PLL Input Range
#pragma config FPLLICLK = PLL_POSC  // System PLL Input Clock Selection
#pragma config FPLLMULT = MUL_50    // System PLL Multiplier (50x Multiplier)
#pragma config FPLLODIV = DIV_2     // System PLL Output Clock Divider (2x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ // USB PLL Input Frequency Selection

// DEVCFG1
#pragma config FNOSC = SPLL         // Oscillator Selection Bits (System PLL)
#pragma config DMTINTV = WIN_127_128 // DMT Count Window Interval
#pragma config FSOSCEN = OFF        // Secondary Oscillator Enable
#pragma config IESO = ON            // Internal/External Switch Over
#pragma config POSCMOD = EC         // Primary Oscillator Configuration
#pragma config OSCIOFNC = OFF       // CLKO Output Signal Active on the OSCO Pin
#pragma config FCKSM = CSECME       // Clock Switching and Monitor Selection
#pragma config WDTPS = PS1048576    // Watchdog Timer Postscaler
#pragma config WDTSPGM = STOP       // Watchdog Timer Stop During Flash Programming
#pragma config WINDIS = NORMAL      // Watchdog Timer Window Mode
#pragma config FWDTEN = OFF         // Watchdog Timer Enable
#pragma config FWDTWINSZ = WINSZ_25 // Watchdog Timer Window Size
#pragma config DMTCNT = DMT31       // Deadman Timer Count Selection
#pragma config FDMTEN = OFF         // Deadman Timer Enable

// DEVCFG0
#pragma config DEBUG = OFF          // Background Debugger Enable
#pragma config JTAGEN = OFF         // JTAG Enable
#pragma config ICESEL = ICS_PGx1    // ICE/ICD Comm Channel Select
#pragma config TRCEN = OFF          // Trace Enable
#pragma config BOOTISA = MIPS32     // Boot ISA Selection
#pragma config FECCCON = OFF_UNLOCKED // Dynamic Flash ECC Configuration
#pragma config FSLEEP = OFF         // Flash Sleep Mode
#pragma config DBGPER = PG_ALL      // Debug Mode CPU Access Permission
#pragma config SMCLR = MCLR_NORM    // Soft Master Clear Enable bit
#pragma config SOSCGAIN = GAIN_2X   // Secondary Oscillator Gain Control bits
#pragma config SOSCBOOST = ON       // Secondary Oscillator Boost Kick Start Enable bit
#pragma config POSCGAIN = GAIN_2X   // Primary Oscillator Gain Control bits
#pragma config POSCBOOST = ON       // Primary Oscillator Boost Kick Start Enable bit
#pragma config EJTAGBEN = NORMAL    // EJTAG Boot

// System clock frequency (188MHz as specified)
#define SYS_FREQ 188000000UL
#define PBCLK_FREQ (SYS_FREQ / 2)  // Peripheral bus clock is typically half of system clock

// UART configuration
#define UART_BAUD_RATE 9600
#define UART_BRG_VALUE ((PBCLK_FREQ / (16 * UART_BAUD_RATE)) - 1)

// Buffer size
#define BUFFER_SIZE 256

// Function prototypes
void UART1_Init(void);
void UART1_WriteByte(char data);
void UART1_WriteString(const char* str);
char UART1_ReadByte(void);
int UART1_DataAvailable(void);
void DelayMs(unsigned int ms);

// Global variables
char rx_buffer[BUFFER_SIZE];
char tx_buffer[BUFFER_SIZE];
volatile int rx_index = 0;

void UART1_Init(void) {
    // Configure UART1 pins
    // Assuming U1TX on RPB15 and U1RX on RPB14 (check your specific pinout)
    ANSELB &= ~(1 << 14 | 1 << 15); // Disable analog on RB14 and RB15
    TRISB &= ~(1 << 15);            // RB15 as output (U1TX)
    TRISB |= (1 << 14);             // RB14 as input (U1RX)
    
    // Configure peripheral pin select for UART1
    SYSKEY = 0x00000000;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    CFGCONbits.IOLOCK = 0;          // Unlock PPS
    
    RPB15R = 1;                     // U1TX on RPB15
    U1RXR = 3;                      // U1RX on RPB14
    
    CFGCONbits.IOLOCK = 1;          // Lock PPS
    SYSKEY = 0x00000000;
    
    // Configure UART1
    U1MODE = 0;                     // Clear UART1 mode register
    U1STA = 0;                      // Clear UART1 status register
    
    // Set baud rate
    U1BRG = UART_BRG_VALUE;
    
    // Configure UART1 mode
    U1MODEbits.STSEL = 0;           // 1 stop bit
    U1MODEbits.PDSEL = 0;           // No parity, 8 data bits
    U1MODEbits.ABAUD = 0;           // Auto-baud disabled
    U1MODEbits.BRGH = 0;            // Standard speed mode
    
    // Enable UART1 transmitter and receiver
    U1STAbits.UTXEN = 1;            // Enable transmitter
    U1STAbits.URXEN = 1;            // Enable receiver
    
    // Enable UART1
    U1MODEbits.ON = 1;
    
    // Enable UART1 receive interrupt
    IPC28bits.U1RXIP = 4;          // Set priority level 4
    IPC28bits.U1RXIS = 0;          // Set subpriority level 0
    IFS3bits.U1RXIF = 0;           // Clear interrupt flag
    IEC3bits.U1RXIE = 1;           // Enable interrupt
    
    // Enable multi-vector interrupts
    INTCONbits.MVEC = 1;
    __builtin_enable_interrupts();
}

void UART1_WriteByte(char data) {
    // Wait for transmit buffer to be empty
    while (U1STAbits.UTXBF);
    U1TXREG = data;
}

void UART1_WriteString(const char* str) {
    while (*str) {
        UART1_WriteByte(*str++);
    }
}

char UART1_ReadByte(void) {
    // Wait for data to be available
    while (!U1STAbits.URXDA);
    return U1RXREG;
}

int UART1_DataAvailable(void) {
    return U1STAbits.URXDA;
}

void DelayMs(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < (SYS_FREQ / 10000); j++) {
            asm("nop");
        }
    }
}

// UART1 receive interrupt service routine
void __ISR(_UART1_RX_VECTOR, IPL4SOFT) UART1_RX_ISR(void) {
    char received_char;
    
    // Read received character
    received_char = U1RXREG;
    
    // Store in buffer
    if (rx_index < BUFFER_SIZE - 1) {
        rx_buffer[rx_index++] = received_char;
        
        // Check for end of message (newline)
        if (received_char == '\n') {
            rx_buffer[rx_index] = '\0';
            // Process received message here if needed
        }
    }
    
    // Clear interrupt flag
    IFS3bits.U1RXIF = 0;
}

int main(void) {
    int counter = 0;
    
    // Initialize UART1
    UART1_Init();
    
    // Send startup message
    UART1_WriteString("PIC32 UART1 initialized\r\n");
    
    while (1) {
        // Check if data received from BeagleBone Black
        if (rx_index > 0) {
            // Echo received data back
            UART1_WriteString("PIC32 received: ");
            UART1_WriteString(rx_buffer);
            
            // Clear receive buffer
            memset(rx_buffer, 0, sizeof(rx_buffer));
            rx_index = 0;
        }
        
        // Send periodic message to BeagleBone Black
        snprintf(tx_buffer, sizeof(tx_buffer), "PIC32 Status: Running, Counter: %d\r\n", counter++);
        UART1_WriteString(tx_buffer);
        
        // Delay before next transmission
        DelayMs(2000);
    }
    
    return 0;
}

// Additional utility functions for debugging
void UART1_SendHex(unsigned int value) {
    char hex_string[9];
    sprintf(hex_string, "%08X", value);
    UART1_WriteString(hex_string);
}

void UART1_SendDecimal(int value) {
    char dec_string[12];
    sprintf(dec_string, "%d", value);
    UART1_WriteString(dec_string);
}