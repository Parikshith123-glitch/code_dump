
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
#include <stdint.h>
#include <stdlib.h>

void osc_init(void);
void i2c_init(void);
//int dac_check_presence(void);

void i2c_start_dac2(void);
void i2c_stop_dac2(void);
void i2c_write_dac2(uint8_t, uint8_t);
uint8_t i2c_read_dac(uint8_t);

void mcp_set(uint8_t port, uint8_t pin, uint8_t state);
void MCP_init(void);

#define address_slave 0x20

int main()
{
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;

    osc_init();
    i2c_init();
    MCP_init();
 
    unsigned long int i = 0;         // Loop Variable 
    
//    TRISAbits.TRISA0 = 0;
//    TRISBbits.TRISB1 = 1;
//    TRISBbits.TRISB2 = 1;
//    
//    ANSELBbits.ANSB1 = 0; // Make RB1 digital
//    ANSELBbits.ANSB2 = 0; // Make RB2 digital
//
//    TRISBbits.TRISB4 = 0;   // CLR Pin Output
    
    while(1)
    {
        mcp_set('B', 3, 0);
        for(i = 0; i< 400000; i++);
        mcp_set('B', 2, 1);
        for(i = 0; i< 400000; i++);
        
        //result = dac_check_presence();
        //LATCbits.LATC1=1;
    }
    return 0;
}

/**************************** Function Definitions ****************************/

void osc_init(void)
{
    OSCFRQbits.HFFRQ = 5; // 16 MHz
}

void i2c_init(void)
{
    SSP2CON1bits.SSPM = 8;      // I2C Master Control enable
    SSP2CON1bits.SSPEN = 1;     // to enable BRG values
    SSP2ADD = 0x09;             // now - 400 KHz setting the BRG value for OSC-FRQ for 92 decimal - i2c frequency as 32khz 0x5C = 92 in hex
}

void i2c_start_dac2(void)
{
    SSP2CON2bits.SEN = 1;
    while(SSP2CON2bits.SEN);
}

void i2c_stop_dac2(void)
{
    SSP2CON2bits.PEN = 1;
    while(SSP2CON2bits.PEN);        // This gets automatically cleared by hardware so after I2C stops,
                                    // it automatically resets back to 0 to break the while loop
}

//int dac_check_presence(void)
//{
//    volatile int ack = 888;
//    
//    i2c_start_dac2();
//    ack = i2c_write_dac2(0x1B);  // 0x1A = (ADDR=VDD) << 1 | 0 for write
//    i2c_stop_dac2();
//
//    return !ack;  // 0 = ACK received => device is present
//}

void i2c_write_dac2(uint8_t address, uint8_t data)
{   
    i2c_start_dac2();
    
    SSP2BUF = address_slave << 1;
    while(SSP2STATbits.BF);         // Buffer Full Bit
    while(SSP2CON2bits.ACKSTAT);
    
    SSP2BUF = address;              // Register Address
    while(SSP2STATbits.BF); 
    while(SSP2CON2bits.ACKSTAT);
    
    SSP2BUF = data;                 // Actual Data
    while(SSP2STATbits.BF); 
    while(SSP2CON2bits.ACKSTAT);

    i2c_stop_dac2();
    
    //return SSP2CON2bits.ACKSTAT;    // 0 = ACK was received, 1 = NACK
}

uint8_t i2c_read_dac(uint8_t reg)
{
    uint8_t received_data;
    
    i2c_start_dac2();
        
    SSP2BUF = (address_slave << 1); // slave address with write bit at the LSB
    while(SSP2STATbits.BF); 
    while(SSP2CON2bits.ACKSTAT);
    
    SSP2BUF = reg;                  // register address
    while(SSP2STATbits.BF); 
    while(SSP2CON2bits.ACKSTAT);
    
    SSP2CON2bits.RSEN = 1;          // restarted start
    while(SSP2CON2bits.RSEN);
    
    SSP2BUF = (address_slave << 1) | 1; // Address + Read
    while(SSP2STATbits.BF); 
    while(SSP2CON2bits.ACKSTAT); // 1 = NACK received, 0 = ACK received
    
    SSP2CON2bits.RCEN = 1;
    while(SSP2STATbits.BF);        // Receive complete indication is opposite in the data sheet 

    received_data = SSP2BUF;
    
    SSP2CON2bits.ACKEN = 1;                      // Send ACK/NACK
    SSP2CON2bits.ACKDT = 1;                      // NACK (no more data expected)
    while (!SSP2CON2bits.ACKEN);                  // Wait until ACK/NACK is sent

    i2c_stop_dac2();                                  // Stop condition
    
    return received_data;
}

/*******************************************************************************/

void MCP_init()
{
    i2c_write_dac2(0x00, 0xF0);
    i2c_write_dac2(0x01, 0x00);
}

void mcp_set(uint8_t port, uint8_t pin, uint8_t state)
{
    uint8_t reg = (port == 'A') ? 0x12 : 0x13; // GPIOA or GPIOB
    uint8_t value = i2c_read_dac(reg);  // Read current value
    
    if (state)
        value |= (1 << pin);  // Set pin high
    else
        value &= ~(1 << pin); // Set pin low
    i2c_write_dac2(reg, value);    // Write back new value
}


