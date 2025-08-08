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
#pragma config FPLLMULT = MUL_36        // System PLL Multiplier (PLL Multiply by 36)
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

// PIC32MZ2048EFH064 Configuration Bit Settings

#include <xc.h>
#include <p32xxxx.h>
#include <stdint.h>

// Function Prototypes
void initiate_key(void);
void delay_ms(uint32_t);

int main()
{   
    // Configure LED pins as outputs
    TRISCbits.TRISC13 = 0;
    TRISCbits.TRISC14 = 0;

    // Turn off LEDs initially
    LATCbits.LATC13 = 1;
    LATCbits.LATC14 = 1;
    
    unsigned int x, i;
    unsigned int rowbuff[512]; // 512 words = 2048 bytes (one flash row)

    // Fill row buffer with data (0 to 2047)
    for (x = 0; x < 512; x++) // here we are filling 512 words which comes out to be 2048 bytes which is equal to 1 Row of bytes.
    {
        rowbuff[x] = x; 
    }
    
    // Flash memory address (in KSEG0 space)
    unsigned int flash_addr = 0x1D008000;
    
    // Erase Flash Page (aligned to 4KB)
    NVMADDR = flash_addr;
    NVMCONbits.NVMOP = 0x4; // Page Erase
    NVMCONbits.WREN = 1;
    initiate_key();
    while (NVMCONbits.WR);
    NVMCONbits.WREN = 0;

    // Program Flash Row
    NVMADDR = flash_addr; // Row address (must be aligned)
    
    // Convert virtual address to physical address
    NVMSRCADDR = (unsigned int)((int)rowbuff & 0x1FFFFFFF);
    
    NVMCONbits.NVMOP = 0x3; // Row Programming
    NVMCONbits.WREN = 1;
    initiate_key();
    while (NVMCONbits.WR);
    NVMCONbits.WREN = 0;
    
    
    unsigned int v_address_claude = flash_addr | 0xA0000000; // not used here (virtual address)
    
    // Erase Flash Page (aligned to 4KB)
    NVMADDR = flash_addr;
    NVMCONbits.NVMOP = 0x4; // Page Erase
    NVMCONbits.WREN = 1;
    initiate_key();
    while (NVMCONbits.WR);
    NVMCONbits.WREN = 0;                    // using flash again 
    
    
    // Use the same flash address in KSEG0 space for verification
    
    volatile uint32_t *ptr = (volatile uint32_t *)v_address_claude; // changed from 8 bit to 32 bit.
    unsigned int actual_sum = 0;
    for (i = 0; i < 512; i++)
    {
        actual_sum += (ptr[i]);
    }
    unsigned int expected_sum = (511*512) / 2;                      // actual sum from 0 to 511 numbers as this is the limit for a row 

    // Check for NVM error
    if (NVMCON & 0x3000) {
        // NVM error - rapidly blink LED13
        while (1) {
            LATCbits.LATC13 = 1;
            delay_ms(50);
            LATCbits.LATC13 = 0;
            delay_ms(50);
        }
    }

    // Compare calculated sum with expected sum
    if (actual_sum == expected_sum) {
        // Success - both LEDs blink together
        while (1) {
            LATCbits.LATC14 = 1;
            LATCbits.LATC13 = 1;
            delay_ms(100);
            LATCbits.LATC14 = 0;
            LATCbits.LATC13 = 0;
            delay_ms(100);
        }
    } else {
        // Verification failed - alternate LED blinking
        while (1) {
            LATCbits.LATC14 = 1;
            delay_ms(50);
            LATCbits.LATC14 = 0;
            delay_ms(50);
            LATCbits.LATC13 = 1;
            delay_ms(50);
            LATCbits.LATC13 = 0;
            delay_ms(50);
        }
    }

    return 0;
}

void initiate_key(void)
{
    unsigned int int_status;
    int dma_susp;

    asm volatile("mfc0 %0,$12" : "=r"(int_status));
    asm volatile("di");
    asm volatile("ehb");

    if (!(dma_susp = DMACONbits.SUSPEND)) {
        DMACONSET = _DMACON_SUSPEND_MASK;
        while (DMACONbits.DMABUSY);
    }

    NVMKEY = 0x0;
    NVMKEY = 0xAA996655;
    NVMKEY = 0x556699AA;
    NVMCONSET = 1 << 15;

    if (!dma_susp) {
        DMACONCLR = _DMACON_SUSPEND_MASK;
    }

    if (int_status & 0x1) {
        asm volatile("ei");
    }
}

void delay_ms(uint32_t ms)
{
    uint32_t cycles_per_ms = 8000;
    for (volatile uint32_t i = 0; i < ms * cycles_per_ms; i++) {
        // intentional empty loop for delay
    }
}