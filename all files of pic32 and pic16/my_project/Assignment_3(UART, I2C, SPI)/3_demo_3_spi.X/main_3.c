
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
#pragma config FPLLIDIV = DIV_4         // System PLL Input Divider (1x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_80        // System PLL Multiplier (PLL Multiply by 47)
#pragma config FPLLODIV = DIV_2        // System PLL Output Clock Divider (2x Divider)
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

// BMP280 SPI communication example (bare-metal, simplified)
#include <xc.h>
#include <p32xxxx.h>
#include <stdint.h>

void SPI1_Init(void);
void BMP280_Select(void);
void BMP280_Deslect(void);
uint8_t SPI1_Transfer(uint8_t data);
uint8_t BMP280_ReadRegister(uint8_t reg_addr);
uint32_t BMP280_ReadRawTemp(void);
uint32_t BMP280_ReadRawPressure(void);

// Define CS pin for BMP280 (adjust as needed)
#define CS_TRIS   TRISDbits.TRISD0 // CS/SS = D0 for this any gpio can be used to do high/low
#define CS_LAT    LATDbits.LATD0   // We can interface any gpio for the CS/SS pin.


unsigned short dig_T1=27504;
short dig_T2 = 26435;
short dig_T3 = -1000;

double bmp280_compensate_T_double(unsigned int adc_T)
{
double var1, var2, T;
var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
//t_fine = (unsigned int)(var1 + var2);
T = (var1 + var2) / 5120.0;
return T;
}




// Main example usage
int main(void) 
{
    unsigned int delay;
    // SCK is default mapped to pin 49 in the data sheet.
    TRISCbits.TRISC13 = 0;      // MOSI = C13 SDO
    TRISCbits.TRISC14 = 1;      // MISO = C14 SDI
    
    SPI1_Init();
    
    while (1) 
    {
        volatile uint32_t raw_temp = BMP280_ReadRawTemp();
        double r= bmp280_compensate_T_double(raw_temp);
        volatile uint32_t raw_press = BMP280_ReadRawPressure();
        //for(delay = 0; delay < 90000; delay++);
        
        uint32_t slave_id = BMP280_ReadRegister(0xD0);
        
    }
    return 0;
}

// SPI initialization
void SPI1_Init(void) 
{
    CS_TRIS = 0;     // Set CS pin as output
    CS_LAT = 1;      // Set CS high (inactive)
    
    RPC13R = 5;      // Pin C13 - SDO
    SDI1R = 7;       // Pin c14 - SDI

    SPI1CON = 0;                     // Reset SPI1 configuration
    SPI1CONbits.MSTEN = 1;          // Master mode
    SPI1CONbits.CKP = 0;            // Clock idle low
    SPI1CONbits.CKE = 1;            // Data changes on falling edge
    SPI1STATbits.SPIROV = 0;        // Clear overflow
    
    SPI1BRG = 0;                   // Set SPI clock speed (adjust as needed)
    
    SPI1CONbits.ON = 1;             // Turn on SPI
}

// Select BMP280 (CS low)
void BMP280_Select(void) 
{
    CS_LAT = 0;
}

// Deselect BMP280 (CS high)
void BMP280_Deselect(void) 
{
    CS_LAT = 1;
}

// SPI send and receive one byte
uint8_t SPI1_Transfer(uint8_t data) 
{
    SPI1BUF = data;                // we are sending the "address" in the buffer to slave and the slave sends the value/data in that address back.
    while (!SPI1STATbits.SPIRBF);  // Wait for data to be filled in the buffer by the slave
    return SPI1BUF;                // Return received byte
}

// Read a register from BMP280
uint8_t BMP280_ReadRegister(uint8_t reg_addr) 
{
    uint8_t value;
    BMP280_Select();                // Chip Select Low for transfer
    SPI1_Transfer(reg_addr | 0x80); // Set MSB=1 for read
                                    // if this 0x80 is not included then it triggers the write operation and writes 0xFF(next line) 
                                    // to the register address, so this must be included to read the value in the register.
    value = SPI1_Transfer(0xFF);    // Dummy byte to read
                                    // we are sending this 0xFF to get the data(the address of the register or the value of the 
                                    // temperature or the pressure) by doing a clock pulse 
    BMP280_Deselect();              // Chip Select High after transfer.
    return value;
}

// Read raw temperature (20-bit)
uint32_t BMP280_ReadRawTemp(void)
{
    uint8_t msb = BMP280_ReadRegister(0xFA);
    uint8_t lsb = BMP280_ReadRegister(0xFB);
    uint8_t xlsb = BMP280_ReadRegister(0xFC);
    return (((uint32_t)msb << 12) | ((uint32_t)lsb << 4) | (xlsb >> 4));
}

// Read raw pressure (20-bit)
uint32_t BMP280_ReadRawPressure(void) 
{
    uint8_t msb = BMP280_ReadRegister(0xF7);
    uint8_t lsb = BMP280_ReadRegister(0xF8);
    uint8_t xlsb = BMP280_ReadRegister(0xF9);
    return (((uint32_t)msb << 12) | ((uint32_t)lsb << 4) | (xlsb >> 4));
}


