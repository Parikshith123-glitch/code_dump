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

#include<xc.h>
#include<stdio.h>

void Oscillator_Init(void);
void Port_Init(void);
void I2C_Init(void);

void I2C_Reference_Setup(uint8_t cmd, uint16_t data);

void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t, uint16_t);



#define SLAVE_ADDR  0x0C 
#define CMD_WRITE   0x18 //0x18
#define TABLE_SIZE  64

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

int i, count;

int main(void)
{
   Oscillator_Init();
   Port_Init();
   I2C_Init();
//   I2C_Reference_Setup(CMD_WRITE, 0x001);
   
   uint16_t data = 0xFFF;
   
/*-----------------------------------------------------------------------------*/
   while(1)
   {
       I2C_Write(CMD_WRITE, data);
       for(i = 0; i<9000; i++);
       I2C_Write(CMD_WRITE, 0x000);       
       for(i = 0; i<9000; i++);
   }
   
 /*-----------------------------------------------------------------------------*/  
//    while (1)
//    { 
//        for (int i = 0; i < TABLE_SIZE; i++) // number of value
//        {
//            I2C_Write(CMD_WRITE, sine_table[i]);
//            for (volatile int d = 0; d < 3000; d++);  // Adjust this for frequency
//        }
//    } 
/*-----------------------------------------------------------------------------*/
//    uint16_t code_full[] = {0xFFF, 0x7FF, 0x3FF, 0x1FF, 0xFF, 0x7F, 0x3F, 0x1F, 0xF, 0x7, 0x3, 0x1, 0x0};
//    uint8_t ind_value = sizeof(code_full)/sizeof(code_full[0]);
//   
//    while (1)
//    {
//        for(count = 0; count < ind_value; count++)
//        {
//            // i2c_clock_test(); 
//            I2C_Write(CMD_WRITE, code_full[count]);   // 0x08 this the working command byte
//            for(i = 0 ;i < 900; i++);
//        }
//        
//        for(count = ind_value - 1; count > 0; count--)
//        {
//            I2C_Write(CMD_WRITE, code_full[count]);   // 0x08 this the working command byte
//            for(i = 0 ;i < 900; i++);
//            // For clock 4MHz and BRG Value 0x26, the i value for the delay of 1ms is 900.
//        }
//    }
 /*-----------------------------------------------------------------------------*/

   return 0;
}

void Oscillator_Init(void)
{
   OSCFRQbits.HFFRQ = 5;        // 4 mhz frequncy   
//   OSCCON2bits.COSC = 0b110;         // select the HFINTOSC
//   OSCCON2bits.CDIV = 0b011;         // frequncy divided by 8
}

void I2C_Start(void)
{
    SSP2CON2bits.SEN = 1;
    while(SSP2CON2bits.SEN);
}

void I2C_Stop(void)
{
    SSP2CON2bits.PEN = 1;
    while(SSP2CON2bits.PEN);
}

void Port_Init(void)
{   
    // 1. Analog disable
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB2 = 0;

    // 2. Set pins as inputs (open-drain)
//    TRISBbits.TRISB1 = 0;
//    TRISBbits.TRISB2 = 0;
    
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;

    // 3. Input mapping
    SSP2CLKPPS = 0x09;  // RB1 ? MSSP2 clock
    SSP2DATPPS = 0x0A;  // RB2 ? MSSP2 data

    // 4. Output mapping
    RB1PPS = 0x17;      // MSSP2 clock ? RB1
    RB2PPS = 0x18;      // MSSP2 data ? RB2   
    
    
    /*
     ?Hey MSSP2, when you're listening, listen at RB2.
And when you're talking, talk through RB2 too.?

// Input (listening)
SSP2DATPPS = 0x0A;  // SDA input from RB2
SSP2CLKPPS = 0x09;  // SCL input from RB1

// Output (talking)
RB2PPS = 0x18;      // SDA output to RB2
RB1PPS = 0x17;      // SCL output to RB1
     */
}


void I2C_Init(void)
{   
    SSP2CON1bits.SSPM  = 0x08;      // I2C Master mode 
    SSP2ADD = 0x09;                 // For 100KHz with 4MHz clock
    SSP2CON1bits.SSPEN = 0x01;      // Enable after configuration
}

void I2C_Write(uint8_t cmd, uint16_t data) 
{    
    uint8_t msb = (data >> 4) & 0xFF;      // Top 8 bits: D11?D4
    uint8_t lsb = (data << 4) & 0xF0;      // Bottom 4 bits in upper nibble
    
    I2C_Start();
    
    SSP2BUF = SLAVE_ADDR << 1 | 0;
    while(SSP2CON2bits.ACKSTAT);
    while(SSP2STATbits.BF);
    
    while (!PIR3bits.SSP2IF);       // Wait for transmission + ACK cycle
    PIR3bits.SSP2IF = 0;
    
    SSP2BUF = cmd;
    while(SSP2CON2bits.ACKSTAT);
    while(SSP2STATbits.BF);
    
    while (!PIR3bits.SSP2IF);       // Wait for transmission + ACK cycle
    PIR3bits.SSP2IF = 0;
    
    SSP2BUF = msb;
    while(SSP2CON2bits.ACKSTAT);
    while(SSP2STATbits.BF);
    
    while (!PIR3bits.SSP2IF);       // Wait for transmission + ACK cycle
    PIR3bits.SSP2IF = 0;
    
    SSP2BUF = lsb;
    while(SSP2CON2bits.ACKSTAT);
    while(SSP2STATbits.BF);
    
    while (!PIR3bits.SSP2IF);       // Wait for transmission + ACK cycle
    PIR3bits.SSP2IF = 0;
    
    I2C_Stop();
}


void I2C_Reference_Setup(uint8_t cmd, uint16_t data) 
{      
    I2C_Start();
    
    SSP2BUF = SLAVE_ADDR << 1 | 0;
    while(SSP2CON2bits.ACKSTAT);
    while(SSP2STATbits.BF);
    
    while (!PIR3bits.SSP2IF);       // Wait for transmission + ACK cycle
    PIR3bits.SSP2IF = 0;
    
    SSP2BUF = 0x38;
    while(SSP2CON2bits.ACKSTAT);
    while(SSP2STATbits.BF);
    
    while (!PIR3bits.SSP2IF);       // Wait for transmission + ACK cycle
    PIR3bits.SSP2IF = 0;
    
    SSP2BUF = 0x00;
    while(SSP2CON2bits.ACKSTAT);
    while(SSP2STATbits.BF);
    
    while (!PIR3bits.SSP2IF);       // Wait for transmission + ACK cycle
    PIR3bits.SSP2IF = 0;
    
    SSP2BUF = 0x01;
    while(SSP2CON2bits.ACKSTAT);
    while(SSP2STATbits.BF);
    
    while (!PIR3bits.SSP2IF);       // Wait for transmission + ACK cycle
    PIR3bits.SSP2IF = 0;
    
    I2C_Stop();
}



/*---------------------------------------------------------------------------------------------------------------------------------------*/

//    TRISBbits.TRISB1 = 0;
//    TRISBbits.TRISB2 = 0;
//    
//    while(1)
//    {
//      LATBbits.LATB1 = 1;
//      LATBbits.LATB2 = 1;
//    
//    for(i = 0 ; i< 9000; i++);
// 
//      LATBbits.LATB1 = 0;
//      LATBbits.LATB2 = 0;    
//    
//    for(i = 0 ; i< 9000; i++);
//    }
