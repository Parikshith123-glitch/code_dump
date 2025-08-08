
// DEVCFG2
#pragma config FPLLIDIV = DIV_1             // System PLL Input Divider
#pragma config FPLLRNG  = RANGE_8_16_MHZ    // System PLL Input Range
#pragma config FPLLICLK = PLL_FRC           // System PLL Input Clock Selection
#pragma config FPLLMULT = MUL_47            // System PLL Multiplier
#pragma config FPLLODIV = DIV_2             // System PLL Output Divisor
#pragma config UPLLFSEL = FREQ_24MHZ        // USB PLL Input Frequency Selection
//#pragma config UPLLEN   = ON              // USB PLL Enable

// DEVCFG1
#pragma config FNOSC    = SPLL              // Oscillator Selection Bits
#pragma config DMTINTV  = WIN_127_128       // DMT Count Window Interval
#pragma config FSOSCEN  = OFF               // Secondary Oscillator Enable
#pragma config IESO     = OFF               // Internal/External Switch Over
    //#pragma config POSCMOD  = EC          // Primary Oscillator
#pragma config POSCMOD  = OFF               // Primary Oscillator
    //#pragma config OSCIOFNC = ON          // CLKO Output Signal
#pragma config OSCIOFNC = OFF               // CLKO Output Signal
#pragma config FCKSM    = CSDCMD            // Clock Switching and Monitor Selection
#pragma config WDTPS    = PS1048576         // Watchdog Timer Postscaler
#pragma config WDTSPGM  = STOP              // Watchdog Timer Stop During Flash Programming
#pragma config WINDIS   = NORMAL            // Watchdog Timer Window Mod
#pragma config FWDTEN   = OFF               // Watchdog Timer Enable
#pragma config FWDTWINSZ= WINSZ_25          // Watchdog Timer Window Size
#pragma config FDMTEN   = OFF               // Deadman Timer Enable

/* DEVCFG0 */
#pragma config DEBUG    = ON                // Background Debugger
#pragma config JTAGEN   = OFF               // JTAG Port
#pragma config ICESEL   = ICS_PGx2          // ICE/ICD Communication Channel
#pragma config FECCCON  = DYNAMIC           //  ECC ENABL
#pragma config CP = OFF                     // Code Protect
/*
#pragma config TRCEN    = OFF               // Trace feature in CPU
#pragma config BOOTISA  = MIPS32            // MIPS32 or MICROMIPS

#pragma config FSLEEP   = OFF               // Flash Sleep Mode
#pragma config DBGPER   = PG_ALL            // Allow CPU access to all permission regions
#pragma config EJTAGBEN = NORMAL            // Normal EJTAG functionality
*/
/*
// DEVCP0

#pragma config_alt FWDTEN=OFF
#pragma config_alt USERID = 0x1234u
 */

#include <stdio.h>
#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>
#include <xc.h>
#include <stdint.h>                      /* For uint32_t definition */
#include <stdbool.h>                     /* For true/false definition */
#include <math.h>
#include <sys/attribs.h>
#include <p32xxxx.h>
#include <sys/attribs.h>




void RTCC_GetTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds) {
    uint32_t time = RTCTIME;
    *hours = (time >> 24) & 0xFF;  //  hours
    *minutes = (time >> 16) & 0xFF; //  minutes
    *seconds = (time>>8) & 0xFF;        //  seconds
}

void RTCC_GetDate(uint8_t *year, uint8_t *month, uint8_t *day, uint8_t *d)  {
    uint32_t date = RTCDATE;
    *year = (date >> 24) & 0xFF;  // Extract year
    *month = (date >> 16) & 0xFF;  // Extract month
    *day = (date >> 8) & 0xFF;    // Extract day
    *d = date & 0xFF;
}
uint8_t BCDToDec(uint8_t value) {
    return ((value >> 4) * 10) + (value & 0x0F);
}

// Function Prototypes 

void UART_INIT(void);
void UART_TRANSMIT(char str3[]);

// Global Variables

char str3[1000];
unsigned int i;


int main()
{
  unsigned long time1=0x23593000;// set time to 010 hr, 15 min, 33 sec
  unsigned long date1=0x25012001;// set date to Monday 20 Jan 2025   
   
  uint8_t hours, minutes, seconds;
  uint8_t year, month, day,d;

    SYSKEY = 0xAA996655; // Unlock sequence
    SYSKEY = 0x556699AA;

       // Configure RTCC
    RTCCONbits.ON = 0; // Disable RTCC
    //while(RTCCON&0x40);
    
    RTCCONbits.RTCWREN=1;
 
    RTCTIME = time1; // Set time (HHMMSS)
    RTCDATE = date1; // Set date (YYMMDD)
    RTCCONbits.ON = 1;  // Enable RTCC
    //while(RTCCON&0x40);
    
    RTCCONbits.RTCWREN=0;
  
    //SYSKEY = 0x0;       // Lock sequence
    UART_INIT();
  
  
    while(1)
    {
        //charc, str[200]= "welcome_scope\n\r";
        RTCC_GetTime(&hours, &minutes, &seconds); 
        hours= BCDToDec(hours);
        minutes= BCDToDec(minutes);
        seconds= BCDToDec(seconds);
        
        RTCC_GetDate(&year, &month, &day, &d);
        
        year= BCDToDec(year);
        month= BCDToDec(month);
        day= BCDToDec(day);
        d= BCDToDec(d);
        sprintf(str3,"Date:- %d/%d/20%d      Time:- %d/%d/%d       \n\r",day,month,year,hours, minutes, seconds);
        UART_TRANSMIT(str3);
        switch(d)
        {
         
            case 0:
                strcpy(str3,"Day :- Sunday   \n\r");
                break; 
            case 1:
                strcpy(str3,"Day :- Monday   \n\r");
                break;
            case 2:
                strcpy(str3,"Day :- Tuesday  \n\r");
                break;  
            case 3:
                strcpy(str3,"Day :- Wednesday  \n\r");
                break; 
            case 4:
                strcpy(str3,"Day :- Thursday  \n\r");
                break; 
           case 5:
                strcpy(str3,"Day :- Friday  \n\r");
                break; 
           case 6:
                strcpy(str3,"Day :- Saturday  \n\r");
                break;      
            default :
                break;
        }
         
        //UART_TRANSMIT(str3);
             
            for( i=0;i<9990000;i++);
            for( i=0;i<9990000;i++); 
            // Delay_ms(1000);
    


          
    }
    return 0;
}

void UART_INIT(void)
{
    U1MODEbits.ON = 0;      // UART is switched OFF for configuration 
    U1MODEbits.BRGH = 0;    // 16x Standard speed mode 
    U1MODEbits.PDSEL = 0;   // Parity and data selection bits (8, N, 1)
    U1MODEbits.STSEL = 0;   // Selection of number of stop bits = 1
    
    U1STAbits.UTXEN = 1;    // Enabling UART Transmitting bit in status register 
    U1STAbits.URXEN = 1;    // Enabling UART Receiving bit in status register
    
    RPC13R = 1;             // As this is PPS, we are setting the Port C Pin 13 as UART Transmit (TX) Port 13 to 5th Port in TTB
    U1RXR = 7;             // As this is PPS, we are setting the Port C Pin 14 as UART Receive  (RX) Port `14 to 4th Pin in TTb

    U1BRG = 611;            // Setting the BRG(Baud Rate Generation) value according to 
                            // system frequency to get proper baud rate(bits/sec) SYSCLK = 188 MHz
    
    U1MODEbits.ON = 1;      // UART is switched ON for transmission
}

void UART_TRANSMIT(char str3[])
{
    while(str3[i] != '\0')
    {
        U1TXREG = str3[i++];
        while(U1STAbits.TRMT == 0);
    }
    i = 0;
}