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
#pragma config FPLLMULT = MUL_36        // System PLL Multiplier (PLL Multiply by 47)
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
#include <stdint.h>

// Function Prototypes
void initiate_key(void);
void delay_ms(uint32_t);
uint8_t program_flash_word(uint32_t address, uint32_t data);

int main()
{
    // Function calling
    
    unsigned int i = 0;
    TRISCbits.TRISC13 = 0;  // Configure RC13 as output for LED
    TRISCbits.TRISC14 = 0;  // Configure RC14 as output for error LED
        
    /*Page erase operation */
    
    NVMCONbits.NVMOP = 0x4; // Page Erase
    NVMADDR = 0x1D008000;   // Address to erase
    NVMCONbits.WREN = 1;
    initiate_key();         // Start erase
    while (NVMCONbits.WR);  // Wait for erase to finish
    NVMCONbits.WREN = 0;
    
    // Data to write
    uint32_t data_to_write = 0x11111111;
    uint32_t flash_address = 0x1D008000;
    
    // Write data to flash
    uint8_t write_status = program_flash_word(flash_address, data_to_write);

    // If there is any errors in writing to the location blink rapidly
    if(write_status != 0) {
        while(1) {
            LATCbits.LATC14 = 1;  // Turn ON error LED
            delay_ms(100);
            LATCbits.LATC14 = 0;  // Turn OFF error LED
            delay_ms(100);
        }
    }
    unsigned int v_add = flash_address | 0xa0000000;    // to get virtual address where our actual data is stored.
    
    // Read the data back for verification
    uint32_t read_data = *(volatile uint32_t*)v_add;
    
    // Verify the data matches what we wrote
    if(read_data == data_to_write) {
        // Success
        while(1) {
            LATCbits.LATC13 = 0;  // Turn ON LED (active low)
            delay_ms(500);
            LATCbits.LATC13 = 1;  // Turn OFF LED (active low)
            delay_ms(500);
        }
    } else {
        // Verification failed - data read doesn't match what was written
        // Flash both LEDs in an alternating pattern
        while(1) {
            LATCbits.LATC13 = 0;  // Turn ON LED13 (active low)
            LATCbits.LATC14 = 1;  // Turn OFF LED14 (active low)
            delay_ms(200);
            LATCbits.LATC13 = 1;  // Turn OFF LED13 (active low)
            LATCbits.LATC14 = 0;  // Turn ON LED14 (active low)
            delay_ms(200);
        }
    }
    
    return 0;
}

/***********************************************************************************
 * FUNCTION NAME    : program_flash_word
 * DESCRIPTION      : This function is the actual call that writes the data in the 
 *                      flash location.
 * ARGUMENTS        : uint32_t, uint32_t
 * RETURNS          : uint8_t
 **********************************************************************************/

uint8_t program_flash_word(uint32_t address, uint32_t data)
{
    NVMADDR = address;        // Address and data from the specific register 
    NVMDATA0 = data;          
    
    NVMCONbits.NVMOP = 0x1;   // set this bit as 1 for Word Write
    NVMCONbits.WREN = 1;      // Enable flash write
    
    initiate_key();           // Start write
    
    while(NVMCONbits.WR);     // Wait for write to finish
    
    NVMCONbits.WREN = 0;      // Disable flash write
    
    // Here we are checking the 12th and 13th bit for errors, i.e. write error and voltage error
    return (NVMCON & 0x3000) ? 1 : 0;   
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
    asm volatile("di %0" : "=r" (int_status));
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
        asm volatile("nop");
    }
}