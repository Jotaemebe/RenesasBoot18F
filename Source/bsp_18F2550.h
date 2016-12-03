#include <stdint.h>
#include <usart.h>
#include <flash.h>

// PIC18F26K22 Configuration Bit Settings
#pragma config PLLDIV   = 2         // (8 MHz crystal)
#pragma config CPUDIV   = OSC1_PLL2
#pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
#pragma config FOSC     = HSPLL_HS  // 20Mhz

#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config PWRT     = OFF
#pragma config BOR      = ON
#pragma config BORV     = 3
#pragma config VREGEN   = ON      //USB Voltage Regulator
#pragma config WDT      = OFF
#pragma config WDTPS    = 32768
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = OFF
#pragma config PBADEN   = OFF
#pragma config STVREN   = ON
#pragma config LVP      = OFF
#pragma config XINST    = OFF       // Extended Instruction Set
#pragma config CP0      = OFF
#pragma config CP1      = OFF
#pragma config CPB      = OFF
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
#pragma config WRTB     = OFF       // Boot Block Write Protection
#pragma config WRTC     = OFF
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
#pragma config EBTRB    = OFF


// ************************************************************************** //

#define MODE    PORTBbits.RB0

#define _XTAL_FREQ 48000000     // 48Mhz CLK
#define __delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000000.0)))
#define __delay_ms(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000.0)))

//Prototipos
//uint8_t SRX(void);
//void STX(uint8_t data);
//void SRXbuff(uint8_t *buffer, uint16_t len);
//void InitializeSystem(void);

void InitializeSystem(void) {
    OSCTUNE = 0x00;
    OSCCON = 0x70; //Primary clock source selected

    ADCON1 |= 0x0F; // Default all pins to digital
    T1CON = 0b01000101; // Habilita Timer1 1:1 (FOSC)

    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_OFF &
              USART_ASYNCH_MODE & USART_EIGHT_BIT &
              USART_CONT_RX & USART_BRGH_LOW, 77);	//SPBRG = 77 PARA 9600 BPS

    TRISA = 0b11111111; // Todo Entradas
    TRISB = 0b11111111; // Todo Entradas
    TRISC = 0b11111111; // Todo Entradas

    /**************************************************************************/
}

void SRXbuff(uint8_t *buffer, uint16_t len) {
    uint16_t i; // Length counter
    uint8_t data;

    for (i = 0; i < len; i++) // Only retrieve len characters
    {
        while (!PIR1bits.RC1IF); // Wait for data to be received
        data = RCREG; // Get a character from the USART
        *buffer = data; // and save in the string
        buffer++; // Increment the string pointer
    }
    /******************************************************************************/
}
uint8_t SRX(void) {
    while (!PIR1bits.RC1IF); // Wait for data to be received
    return RCREG; // Get a character from the USART
}
// ************************************************************************** //
void STX(uint8_t data){
    while (BusyUSART());
    WriteUSART(data);
    while (BusyUSART());
}
// ************************************************************************** //
void ClearFlash(void) {
    unsigned short long addr;

    if (SRX() != 0xD0) return;


    for (addr = 0x000800; addr < 0x007FFF; addr += 64) {
        TBLPTR = addr;
        EECON1bits.EEPGD = 1; // point to Flash program memory
        EECON1bits.CFGS = 0; // access Flash program memory
        EECON1bits.WREN = 1; // enable write to memory
        EECON1bits.FREE = 1; // enable Row Erase operation

        EECON2 = 0x55; //write 55h
        EECON2 = 0xAA; // write 0AAh
        EECON1bits.WR = 1; //start erase (CPU stall)
    }
}
// ************************************************************************** //