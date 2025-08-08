
// PIC16F15356 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF        // WDT operating mode (WDT enabled regardless of sleep; SWDTEN ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register not write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF not write protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

// Function Prototypes

void osc_init(void);
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t);
void dac_write(uint16_t);
void enable_internal_reference(void);

int main()
{   
    // Function calling 
    osc_init();
    
    TRISBbits.TRISB1 = 1;       // Set RB1 (SCL) and RB2 (SDA) as inputs
    TRISBbits.TRISB2 = 1;
    
    ANSELBbits.ANSB1 = 0;       // Disable analog function on RB1 and RB2
    ANSELBbits.ANSB2 = 0;

    RB1PPS = 0x21;              // Output MSSP2 SCL to RB1
    RB2PPS = 0x22;              // Output MSSP2 SDA to RB2
    
    SSP2CLKPPS = 0x09;          // Set SCL input as RB1 (0x09)
    SSP2DATPPS = 0x0A;          // Set SDA input as RB2 (0x0A)
    
    i2c_init();
    //enable_internal_reference();
    
    


    while(1)
    {
        dac_write(2047);  // Mid-scale for 12-bit DAC (2047)
    };
    return 0;
}

/***********************************************************************************
 * FUNCTION NAME    : osc_init
 * DESCRIPTION      : This function is for the oscillator initialization for proper 
 *                      frequency
 * ARGUMENTS        : None
 * RETURNS          : void
 **********************************************************************************/

void osc_init(void)
{
    OSCFRQbits.HFFRQ = 4;       // Actual frequency of the Oscillator  4 = 12MHz
}

/*************************************************** ********************************
 * FUNCTION NAME    : i2c_init
 * DESCRIPTION      : This function is for I2C communication initialization to send
 *                      the user data to DAC.
 * ARGUMENTS        : None
 * RETURNS          : void
 **********************************************************************************/

void i2c_init(void)
{
    SSP2CON1bits.SSPM = 8;      // I2C Master Control enable
    SSP2CON1bits.SSPEN = 1;     // to enable BRG values
    SSP2ADD = 0x1D; // 1D = 29 for 100 KHz    // setting the BRG value for OSC-FRQ for 92 decimal - i2c frequency as 32KHz 0x5C = 92 in hex
}

/***********************************************************************************
 * FUNCTION NAME    : i2c_start
 * DESCRIPTION      : This function is to start the i2c communication
 * ARGUMENTS        : None
 * RETURNS          : void
 **********************************************************************************/

void i2c_start(void)
{
    SSP2CON2bits.SEN = 1;          // start enable condition
    while(SSP2CON2bits.SEN);       // Wait until START condition is complete (SEN cleared by hardware)
}

/***********************************************************************************
 * FUNCTION NAME    : i2c_stop
 * DESCRIPTION      : This function is to stop the i2c communication
 * ARGUMENTS        : None
 * RETURNS          : void
 **********************************************************************************/

void i2c_stop(void)
{
    SSP2CON2bits.PEN = 1;       // stop enable condition
    while(SSP2CON2bits.PEN);    // // Wait until STOP condition is complete (PEN cleared by hardware)
}

/***********************************************************************************
 * FUNCTION NAME    : i2c_write
 * DESCRIPTION      : This function is to write the user data in the buffer for transmission
 * ARGUMENTS        : uint8_t
 * RETURNS          : void
 **********************************************************************************/

void i2c_write(uint8_t data)
{
    SSP2BUF = data;             // Transmitting the address/data
    
    while(SSP2STATbits.BF);         // buffer full bit
    while(!PIR3bits.SSP2IF);        // interrupt flag enable for transfer completion bit
    PIR3bits.SSP2IF = 0;        // clearing the interrupt flag after 
    while(SSP2CON2bits.ACKSTAT); // wait until the ack is received ( 0 = ack received, 1 = not received )
} 

/***********************************************************************************
 * FUNCTION NAME    : dac_write
 * DESCRIPTION      : This function 
 * ARGUMENTS        : uint16_t
 * RETURNS          : void
 **********************************************************************************/

//void dac_write(uint16_t value)
//{
//    i2c_start();
//    i2c_write(0x1A);     // I2C address with ADDR = GND // slave address
//    i2c_write(0x18);     // Command: Write and update DAC A  // register address based on ADDR settings 
//    i2c_write((value >> 4)& 0xFF);
//    i2c_write((value << 4)& 0xF0);
//    i2c_stop();                       
//}


void dac_write(uint16_t value)
{
    // Ensure value is 12-bit
    value &= 0x0FFF;
    
    i2c_start();
    i2c_write(0x1E);        // Device address + write bit (0x0F << 1)
    i2c_write(0x1C);        // Command: Write and update DAC (immediate update)
    
    // First byte: command bits (0x1C) are already sent separately,
    // now send 4 MSBs + 4 dummy bits (since command was sent separately)
    i2c_write((value >> 8) & 0x0F);  // Send 4 MSBs
    
    // Second byte: 8 LSBs
    i2c_write(value & 0xFF);
    
    i2c_stop();
}

void enable_internal_reference() 
{
    i2c_start();
    i2c_write(0x1E);       // Slave address (ADDR=VDD, Write)
    i2c_write(0xE1);       // Command: Reference ON (111 + 0000 + DB0=1)
    i2c_write(0x00);       // Unused bits
    i2c_write(0x00);       // Unused bits
    
    i2c_stop();
}

