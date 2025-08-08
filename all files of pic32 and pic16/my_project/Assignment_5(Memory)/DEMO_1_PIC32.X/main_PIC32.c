// PIC32MZ2048EFH064 Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config FMIIEN = OFF              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = OFF              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config PGL1WAY = OFF             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = OFF            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = OFF             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = OFF            // USB USBID Selection (Controlled by the USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // System PLL Input Divider (2x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_10        // System PLL Multiplier (PLL Multiply by 6)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (4x Divider)
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
#pragma config FDMTEN = OFF              // Deadman Timer Enable (Deadman Timer is enabled)

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
#include <stdint.h>
#include <proc/p32mz2048efh064.h>

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);

void dac_write(uint8_t, uint16_t);

void switch_ON_OFF(void);
void internal_reference_setup(void);
void LDAC_setup(void);

void i2c_clock_test(void);
void port_config(void);

#define DAC_ADDR                0x0C              // 7-bit address - ADDR Pin to VCC
#define CMD_WRITE               0x18              // Getting waveforms for this value - 0x08

int i, count;

// Test running for sine wave generation

#define TABLE_SIZE 64

uint16_t sine_table[TABLE_SIZE] = {
    0x800, 0x8C8, 0x98F, 0xA52, 0xB0F, 0xBC5, 0xC71, 0xD12,
    0xDA7, 0xE2E, 0xEA6, 0xF0D, 0xF63, 0xFA7, 0xFD8, 0xFF5,
    0xFFF, 0xFF5, 0xFD8, 0xFA7, 0xF63, 0xF0D, 0xEA6, 0xE2E,
    0xDA7, 0xD12, 0xC71, 0xBC5, 0xB0F, 0xA52, 0x98F, 0x8C8,
    0x800, 0x737, 0x670, 0x5AD, 0x4F0, 0x43A, 0x38E, 0x2ED,
    0x258, 0x1D1, 0x159, 0x0F2, 0x09C, 0x058, 0x027, 0x00A,
    0x000, 0x00A, 0x027, 0x058, 0x09C, 0x0F2, 0x159, 0x1D1,
    0x258, 0x2ED, 0x38E, 0x43A, 0x4F0, 0x5AD, 0x670, 0x737
};


uint16_t sawtooth_table[TABLE_SIZE] = {
    0x000, 0x040, 0x080, 0x0C0, 0x100, 0x140, 0x180, 0x1C0,
    0x200, 0x240, 0x280, 0x2C0, 0x300, 0x340, 0x380, 0x3C0,
    0x400, 0x440, 0x480, 0x4C0, 0x500, 0x540, 0x580, 0x5C0,
    0x600, 0x640, 0x680, 0x6C0, 0x700, 0x740, 0x780, 0x7C0,
    0x800, 0x83F, 0x87F, 0x8BF, 0x8FF, 0x93F, 0x97F, 0x9BF,
    0x9FF, 0xA3F, 0xA7F, 0xABF, 0xAFF, 0xB3F, 0xB7F, 0xBBF,
    0xBFF, 0xC3F, 0xC7F, 0xCBF, 0xCFF, 0xD3F, 0xD7F, 0xDBF,
    0xDFF, 0xE3F, 0xE7F, 0xEBF, 0xEFF, 0xF3F, 0xF7F, 0xFBF
};


int main()
{    
//    port_config();
    i2c_init();
//    switch_ON_OFF();
//    internal_reference_setup();
//    LDAC_setup();
    
/*------------------------ Checking for sine/sawtooth/triangle wave generation using the table -----------------------------------------------------------------------------------*/
    while (1)
{
    for (int i = 0; i < TABLE_SIZE; i++) // number of value 
    {
        dac_write(CMD_WRITE, sine_table[i]);
        for (volatile int d = 0; d < 3000; d++);  // Adjust this for frequency
    }
}    
/*------------------------ Checking for sine wave generation -----------------------------------------------------------------------------------*/

/* ----------------------------------- This below works properly for the 12 bits generation in the oscilloscope ------------------------------*/    
   
//    uint16_t code_full[] = {0xFFF, 0x7FF, 0x3FF, 0x1FF, 0xFF, 0x7F, 0x3F, 0x1F, 0xF, 0x7, 0x3, 0x1, 0x0};
//    //uint16_t code_full[] = {0xFFF, 0x3FF, 0xFF, 0x3F, 0xF, 0x3, 0x0};
//   
//    uint8_t ind_value = sizeof(code_full)/sizeof(code_full[0]);
//   
//    while (1)
//    {
//        for(count = 0; count < ind_value; count++)
//        {
//            // i2c_clock_test(); 
//            dac_write(CMD_WRITE, code_full[count]);   // 0x08 this the working command byte
//            for(i = 0 ;i < 900; i++);
//        }
//        
//        for(count = ind_value - 1; count > 0; count--)
//        {
//            dac_write_2(CMD_WRITE, code_full[count]);   // 0x08 this the working command byte
//            for(i = 0 ;i < 900; i++);
//            // For clock 4MHz and BRG Value 0x26, the i value for the delay of 1ms is 900.
//        }
//    }
    
/* ----------------------------------- This above works properly for the 12 points ----------------------------------------------*/
    
    return 0;
}

/******************************************************************************
FUNCTION NAME : port_config
DESCRIPTION   : I2C1 Pins Configuration Pin 43 - Data(SDA1), Pin 44 - Clock(SCK1)
RETURNS       : void
******************************************************************************/

void port_config(void)
{   
    // Configure I2C pins as inputs (open-drain)
    TRISDbits.TRISD10 = 1;  // SCL - clock line - pin 44
    TRISDbits.TRISD9 = 1;   // SDA - data line - pin 43
}

/******************************************************************************
FUNCTION NAME : i2c_init
DESCRIPTION   : I2C1 communication 7-bit addressing, Baud rate of 100KHz = 0x31, 
              : Oscillator frequency of 10MHz
RETURNS       : void
******************************************************************************/

void i2c_init(void)
{
    I2C1CON = 0;
    I2C1CONbits.SIDL = 0;         // continue to run in idle mode
    I2C1CONbits.A10M = 0;         // Using a 7 bit slave address
    I2C1CONbits.DISSLW = 0;       // slew rate mode disabled for standard mode(100KHz)
    I2C1BRG = 0x31;//0x26;//0x31 for 100KHz and clock of 10MHz;   // Oscillator Frequency = 6MHz and BRG value = 0x26 for 50KHz I2C Frequency
    I2C1CONbits.ON = 1;
}

/******************************************************************************
FUNCTION NAME : i2c_start
DESCRIPTION   : Starts the I2C communication
RETURNS       : void
******************************************************************************/

void i2c_start(void)
{
    I2C1CONbits.SEN = 1;          // Send START condition
    while(I2C1CONbits.SEN);       // Wait for START to complete
}

/******************************************************************************
FUNCTION NAME : i2c_stop
DESCRIPTION   : Stop the I2C communication
RETURNS       : void
******************************************************************************/

void i2c_stop(void)
{
//    I2C1CONbits.PEN = 1;    // Send STOP condition
//    while(I2C1CONbits.PEN); // Wait for STOP to complete

    unsigned int i;
    I2C1CONbits.ON = 0;             // stop  i2c module
    for (i = 0; i < 100; i ++);
    TRISDbits.TRISD9= 0;           // sda  as OUTPUT
    TRISDbits.TRISD10 = 0;           // SCL  AS OUTPUT
    for (i = 0; i < 100; i ++);
    PORTDbits.RD9 = 0;              // SDA
    PORTDbits.RD10 = 1;              // SCL
    I2C1CONbits.ON = 1;             // stop  i2c module
    for (i = 0; i < 1; i ++);
}

void switch_ON_OFF(void)
{
    i2c_start();
    
    I2C1TRN = DAC_ADDR << 1 | 0;         // DAC_ADDR = 0x0C
    while (I2C1STATbits.TRSTAT);         // Wait for transmission complete
    while (I2C1STATbits.ACKSTAT);
    
    I2C1TRN = 0x20;                     // setting the command byte for reference to be ON               
    while (I2C1STATbits.TRSTAT);         
    while (I2C1STATbits.ACKSTAT);
    
    I2C1TRN = 0x00;                     // Upper 4 Bits 
    while (I2C1STATbits.TRSTAT);         
    while (I2C1STATbits.ACKSTAT);
    
    I2C1TRN = 0x01;                      // Lower 8 Bits 
    while (I2C1STATbits.TRSTAT);         
    while (I2C1STATbits.ACKSTAT);

    i2c_stop();
}

void internal_reference_setup(void)
{
    i2c_start();
    
    I2C1TRN = DAC_ADDR << 1 | 0;         // DAC_ADDR = 0x0C
    while (I2C1STATbits.TRSTAT);         // Wait for transmission complete
    while (I2C1STATbits.ACKSTAT);
    
    I2C1TRN = 0x38;                     // setting the command byte for reference to be ON               
    while (I2C1STATbits.TRSTAT);         
    while (I2C1STATbits.ACKSTAT);
    
    I2C1TRN = 0x00;                     // Upper 4 Bits 
    while (I2C1STATbits.TRSTAT);         
    while (I2C1STATbits.ACKSTAT);
    
    I2C1TRN = 0x01;                      // Lower 8 Bits 
    while (I2C1STATbits.TRSTAT);         
    while (I2C1STATbits.ACKSTAT);

    i2c_stop();
}

void LDAC_setup(void)
{
    i2c_start();
    
    I2C1TRN = DAC_ADDR << 1 | 0;         // DAC_ADDR = 0x0C
    while (I2C1STATbits.TRSTAT);         // Wait for transmission complete
    while (I2C1STATbits.ACKSTAT);
    
    I2C1TRN = 0x30;                     // setting the command byte for LDAC update
    while (I2C1STATbits.TRSTAT);         
    while (I2C1STATbits.ACKSTAT);
    
    I2C1TRN = 0x00;                     // Upper 4 Bits 
    while (I2C1STATbits.TRSTAT);         
    while (I2C1STATbits.ACKSTAT);
    
    I2C1TRN = 0x01;                      // Lower 8 Bits 
    while (I2C1STATbits.TRSTAT);         
    while (I2C1STATbits.ACKSTAT);

    i2c_stop();
}

// Test for clock pulses
void i2c_clock_test(void)
{
    i2c_start();
    
    I2C1TRN = 0XAA;
    while (I2C1STATbits.TRSTAT);         // Wait for transmission complete
    
    i2c_stop();
}

/******************************************************************************
FUNCTION NAME : dac_write
DESCRIPTION   : This function takes the command byte, and data from the main and 
 *            : sending the msb first and the lsb.
RETURNS       : void
******************************************************************************/

void dac_write(uint8_t CMD, uint16_t DATA) 
{
    uint8_t MSB = (DATA >> 4) & 0xFF;
    uint8_t LSB = (DATA << 4) & 0xF0;
    
    i2c_start();
    
    I2C1TRN = DAC_ADDR << 1 | 0;         // DAC_ADDR = 0x0C
    while (I2C1STATbits.TRSTAT);         // Wait for transmission complete
    while (I2C1STATbits.ACKSTAT);
    
    I2C1TRN = CMD;                  // 0x08 this the working command byte
    while (I2C1STATbits.TRSTAT);         // Wait for transmission complete
    while (I2C1STATbits.ACKSTAT);
    
    I2C1TRN = MSB;                       // Upper 8 Bits 
    while (I2C1STATbits.TRSTAT);         // Wait for transmission complete
    while (I2C1STATbits.ACKSTAT);
    
    I2C1TRN = LSB;                       // Lower 4 Bits 
    while (I2C1STATbits.TRSTAT);         // Wait for transmission complete
    while (I2C1STATbits.ACKSTAT);

    i2c_stop();
}