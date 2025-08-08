
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
#pragma config FPLLMULT = MUL_32        // System PLL Multiplier (PLL Multiply by 32)
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
#include <stdint.h>

// Function Prototypes

void delay_ms(uint32_t ms);
void initiate_key();

int main()
{
    TRISCbits.TRISC13 = 0;
    TRISCbits.TRISC14 = 0;
    
    LATCbits.LATC13 = 1;
    LATCbits.LATC14 = 1;
    
    // Erase Initially
    
    NVMCONbits.NVMOP = 0x4; // Page Erase
    NVMADDR = 0x1D008000;   // Address to erase
    NVMCONbits.WREN = 1;
    
    initiate_key();         // Start erase
    
    while (NVMCONbits.WR);  // Wait for erase to finish
    NVMCONbits.WREN = 0;
    
     // Set up Address and Data Registers
    //NVMADDR = 0x1D008000;     // physical address
    
    NVMDATA0 = 0x11111111;     // value written to 0x1D008000
    NVMDATA1 = 0x22222222;     // value written to 0x1D008004
    NVMDATA2 = 0x33333333;     // value written to 0x1D008008
    NVMDATA3 = 0x44444444;     // value written to 0x1D00800C

    uint32_t data1 = 0x11111111;
    uint32_t data2 = 0x22222222;
    uint32_t data3 = 0x33333333;
    uint32_t data4 = 0x44444444;
    
    // set the operation, assumes WREN = 0
    NVMCONbits.NVMOP = 2;       // 2 for Quad Word programming
                                // Enable Flash for write operation and set the NVMOP 
    NVMCONbits.WREN = 1;        
 
    initiate_key();

    while(NVMCONbits.WR);

    NVMCONbits.WREN = 0;
 
    if(NVMCON & 0x3000) // mask for WRERR and LVDERR bits
    {
 // process errors
        LATCbits.LATC13 = 1;
        delay_ms(500);
        LATCbits.LATC13 = 0;
        delay_ms(500);

    }
    
    // getting the virtual address
    unsigned int v1 = 0x1D008000 | 0xA0000000;
    unsigned int v2 = 0x1D008004 | 0xA0000000;
    unsigned int v3 = 0x1D008008 | 0xA0000000;
    unsigned int v4 = 0x1D00800C | 0xA0000000;
    
    // Getting the values from the virtual address
    
    uint32_t read1 = *(volatile uint32_t*)v1;
    uint32_t read2 = *(volatile uint32_t*)v2;
    uint32_t read3 = *(volatile uint32_t*)v3;
    uint32_t read4 = *(volatile uint32_t*)v4;
    
    while(1)
    {
    if(read1 == data1 && read2 == data2 && read3 == data3 && read4 == data4)     
        {     // if this verification works then the data written is correct
        LATCbits.LATC14 ^= 1; // // if right then toggle the GREEN LED rapidly
        delay_ms(50);
        } else {
        LATCbits.LATC13 ^= 1; // if wrong then toggle the RED LED rapidly
        delay_ms(50);
        }
    }
    
return 0;
}

/***********************************************************************************
 * FUNCTION NAME    : initiate_key
 * DESCRIPTION      : This function holds a sequence of keys needed to unlock the location 
 *                      to write in the flash memory
 * ARGUMENTS        : void
 * RETURNS          : void
 **********************************************************************************/

void initiate_key(void)
{
    int int_status;             // storage for current Interrupt Enable state
    int dma_susp;               // storage for current DMA state
    // Disable Interrupts
    asm volatile("di");
    asm volatile("ehb");
    // Disable DMA
    if(!(dma_susp=DMACONbits.SUSPEND))
    {
        DMACONSET=_DMACON_SUSPEND_MASK;     // suspend
        while((DMACONbits.DMABUSY));        // wait to be actually suspended
    }
    
    NVMKEY = 0x0;
    NVMKEY = 0xAA996655;
    NVMKEY = 0x556699AA;
    NVMCONSET = 1 << 15;                    // this it the operation that does the re-writing of the data in the mentioned location.
                                            // Restore DMA
    if(!dma_susp)
    {
        DMACONCLR=_DMACON_SUSPEND_MASK;     // resume DMA activity
    }
    // Restore Interrupts
    if(int_status & 0x00000001) 
    {
        asm volatile("ei");
    }
}

/***********************************************************************************
 * FUNCTION NAME    : delay_ms
 * DESCRIPTION      : This is just a small delay function
 * ARGUMENTS        : uint32_t
 * RETURNS          : void
 **********************************************************************************/

void delay_ms(uint32_t ms)
{
    // At 32MHz CPU clock, approximately 8000 cycles per ms
    // Adjust this value based on your actual clock frequency
    uint32_t cycles_per_ms = 8000;
    
    for(volatile uint32_t i = 0; i < ms * cycles_per_ms; i++)
    {
        // NOP to prevent optimization
        //asm volatile("nop");
    }
}