
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

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t);
uint8_t i2c_read(uint8_t);

void bmp_init(void);
uint32_t bmp_read_temp(void);
uint8_t bmp_id_check(void);

#define BMP_ADDR 0x77 // sdo ground - 76, sdo to vcc(3.3) - 77 

int main()
{
    i2c_init();
    bmp_init();
   
    char i = 0;
    uint32_t raw_temp_main;
    uint8_t id = bmp_id_check();	// optional check for verification.
    
    while(1)
    {
        raw_temp_main = bmp_read_temp();
        //for(i = 0; i < 90000; i++);
    }
    return 0;
}

void i2c_init(void)
{
    I2C1CONbits.ON = 0;
    I2C1BRG = 2;
    I2C1CONbits.A10M = 0;
    I2C1CONbits.ON = 1;
}

void i2c_start(void)
{
    I2C1CONbits.SEN = 1;
    while(I2C1CONbits.SEN);
}

void i2c_stop(void)
{
    I2C1CONbits.PEN = 1;
    while(I2C1CONbits.PEN);
}

uint8_t i2c_read(uint8_t acknowledge)
{
    I2C1CONbits.RCEN = 1;
    while(!I2C1STATbits.RBF);           // wait until the receive buffer is full 
    uint8_t data = I2C1RCV;
    I2C1CONbits.ACKDT = acknowledge;    // if this is 0 then send more bytes of data i.e. after every 8 bits if it is 1 then don't send any more data
    I2C1CONbits.ACKEN = 1;              // this is enabled to start the ack process i.e. sending the ack data.
    while(I2C1CONbits.ACKEN);           // this breaks when the acken turns to 0;
    return data;
}

void i2c_write(uint8_t data)
{
    // we are writing a address or data and putting it in the transmit register 
    I2C1TRN = data;
    while (I2C1STATbits.TRSTAT);        // wait for transmit status 
   // while (I2C1STATbits.ACKSTAT);       // wait for ack bit 
}
void bmp_init(void)
{
    // digital sensors may be in sleep mode so we are initializing the sensor to read the temp and pressure values.
    i2c_start();
    i2c_write((BMP_ADDR << 1) | 0); // we are setting the last bit as 0 so that we can write to this 
    i2c_write(0xF4);    // address of the register where the pressure and temperature is present 
    i2c_write(0x27);    // temp and pressure 
    i2c_stop();
}

uint32_t bmp_read_temp(void)
{
    uint8_t msb, lsb, xlsb;
    i2c_start();
    i2c_write((BMP_ADDR << 1) | 0);	// dev address with write condition
    i2c_write(0xFA);			// register address
    
    i2c_start();			
    i2c_write((BMP_ADDR << 1 ) | 1);	// dev address with read condition
    msb = i2c_read(0);			// data byte to be read (in BMP280 it gives of 20 bits so in the bottom we are converting it to 32 bits for proper reading)
    lsb = i2c_read(0);
    xlsb = i2c_read(1);
    i2c_stop();
    
    uint32_t raw_temp = ((msb << 12) | (lsb << 4) | (xlsb >> 4));
    return raw_temp;
}

// Optional Check //

uint8_t bmp_id_check(void)
{
    i2c_start();
    i2c_write((BMP_ADDR << 1) | 0);// write to the next address
    i2c_write(0xD0);
    
    i2c_start();
    i2c_write((BMP_ADDR << 1) | 1);// write to the next address
    uint8_t chip_id = i2c_read(1);
    i2c_stop();
    return chip_id;
}