
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
#include <stdint.h>

#define MCP23017_ADDR  0x20 

int main(void) 
{
    I2C1_Init();
    MCP23017_Init();
    uint8_t i = 0;
    
    while (1) 
    {
        void LED_A_ON(void);
        for(i = 0; i < 900000; i++);
        void LED_B_ON(void);
        for(i = 0; i < 900000; i++);
        
        void LED_A_OFF(void);
        for(i = 0; i < 900000; i++);        
        void LED_B_OFF(void);
        for(i = 0; i < 900000; i++);

    }
    return 0;
}

void I2C1_Init(void) 
{
    I2C1BRG = 2;                // Set baud rate generator
    I2C1CONbits.A10M = 0;
    I2C1CONbits.ON = 1;         // Enable I2C1 module
}

void I2C1_Start(void) 
{
    I2C1CONbits.SEN = 1;        // Send Start bit
    while (I2C1CONbits.SEN);    // Wait for completion
}

void I2C1_Stop(void) 
{
    I2C1CONbits.PEN = 1;        // Send Stop bit
    while (I2C1CONbits.PEN);    // Wait for completion
}

void I2C1_Write(uint8_t data) 
{
    I2C1TRN = data;                 // Load data
    while (I2C1STATbits.TRSTAT);    // Wait for transmission
    while (I2C1STATbits.ACKSTAT);   // Wait for ACK (0 = ACK)
}

// MCP23017 Initialization - Set Port A as output
void MCP23017_Init(void) 
{
    I2C1_Start();
    I2C1_Write((MCP23017_ADDR << 1) | 0); // Write operation
    I2C1_Write(0x00);   // send the address of the iodira register to talk to (direction control)
    I2C1_Write(0x00);   // set all the pins as outputs / write it to 0
    I2C1_Stop();
}

void LED_A_ON(void)
{
    I2C1_Start();
    I2C1_Write((MCP23017_ADDR << 0) | 0 );
    I2C1_Write(0x14);
    I2C1_Write(0xFF);
    I2C1_Stop();
}

void LED_A_OFF(void)
{
    I2C1_Start();
    I2C1_Write((MCP23017_ADDR << 0) | 0 );
    I2C1_Write(0x14);
    I2C1_Write(0x00);
    I2C1_Stop();
}


void LED_B_ON(void)
{
    I2C1_Start();
    I2C1_Write((MCP23017_ADDR << 0) | 0 );
    I2C1_Write(0x15);
    I2C1_Write(0xFF);
    I2C1_Stop();
}
 
void LED_B_OFF(void)
{
    I2C1_Start();
    I2C1_Write((MCP23017_ADDR << 0) | 0 );
    I2C1_Write(0x15);
    I2C1_Write(0x00);
    I2C1_Stop();
}

/******************** TO READ FROM THE I/O EXPANDER ***********************************/

uint8_t i2c_read()
{
    uint8_t data = 0;
    
    I2C1CONbits.RCEN = 1;
    I2C1CONbits.ACKDT = 1;
    I2C1CONbits.ACKEN = 1;          // 
    while(!I2C1STATbits.RBF);       // wait until the receive buffer is full    
    data = I2C1RCV;                 // get the data from the buffer and send it back 
    
    return data;
}

uint8_t MCP23017_Read(void)
{
    uint8_t data;

    I2C1_Start();
    I2C1_Write((MCP23017_ADDR << 1) | 0); // Write mode
    I2C1_Write(0x12);                     // first need to write the address and then read from that address
    
    I2C1_Start();                         // Repeated start for the slave to send the data 
    I2C1_Write((MCP23017_ADDR << 1) | 1); // Read mode
    data = i2c_read();                   // Read GPIOA data
    I2C1_Stop();

    return data;
}
