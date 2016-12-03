#include <htc.h>

#if defined(__18F2550)
#include "bsp_18F2550.h"
#elif defined(__18F26K22)
#include "bsp_18F26K22.h"
#endif

uint8_t SRD1 = 0x80;
uint8_t SRD2 = 0x00;
uint8_t RXBUF[256];
uint8_t KeyBuff[11];

uint24_t GlobalAddr;
uint24_t GlobalAddr2;


void SendVersion(void);
void UnlockKEY(void);
void ProgramPage(void);
void ReadPage(void);
void ClearFlash(void);

/******************************************************************************/
// Interrupt

void ISRH(void) @ 0x0008 {
    asm("GOTO	0x0802"); // ReMap de 0x0008
}
// ************************************************************************** //

void ISRL(void) @ 0x0018 {
    asm("GOTO	0x0804"); // ReMap de 0x0018
}
// ************************************************************************** //

void main(void) {
    unsigned char i;

    InitializeSystem();

    GlobalAddr = 0x800;
    GlobalAddr2 = 0x800;

    while (1) {
        i = SRX();

        switch (i) {
            case 0x00: // NULL Command
                break;
            case 0x41: // Program PAGE
                ProgramPage();
                break;
            case 0xB0: // Baud to 9600
                STX(i);
                SPBRG = 77;
                break;
            case 0xB1: // Baud to 19200
                STX(i);
                break;
            case 0xB2: // Baud to 38400
                STX(i);
                break;
            case 0xB3: // Baud to 57600
                STX(i);
                SPBRG = 12;
                break;
            case 0xB4: // Baud to 115200
                STX(i);
                break;
            case 0x50: // Clear Status Bits
                SRD1 = 0x80;
                SRD2 = 0x00;
                GlobalAddr = 0x800;
                GlobalAddr2 = 0x800;
                break;
            case 0x70: // Send Status Bytes
                //__delay_ms(1);
                STX(SRD1);
                STX(SRD2);
                break;
            case 0xA7:
                ClearFlash();
                break;
            case 0xF5: // Unlock KEY
                UnlockKEY();
                break;
            case 0xFB: // Bootloader Version
                SendVersion();
                break;
            case 0xFF: // Page READ
                ReadPage();
                break;
        }

    }
    // Aqui nunca llegamos, solo incluimos las funciones de remapeo
    ISRH();
    ISRL();
}

// ************************************************************************** //

/* User Functions                                                             */

void SendVersion(void) {
    STX('V');
    STX('E');
    STX('R');
    STX('.');
    STX('1');
    STX('.');
    STX('0');
    STX('0');
}
// ************************************************************************** //

void UnlockKEY(void) {
    //uint8_t KeyCheck[11] = {0xDF, 0xFF, 0x00, 0x07, 0x60, 0x00, 0x01, 0x14, 0x46, 0x4D, 0x52};

    SRXbuff(KeyBuff, 11);

    if ((KeyBuff[0] == 0xDF) &&
            (KeyBuff[1] == 0xFF) &&
            (KeyBuff[2] == 0x00) &&
            (KeyBuff[3] == 0x07)) {

        if ((KeyBuff[4] == 0x60) &&
                (KeyBuff[5] == 0x00) &&
                (KeyBuff[6] == 0x01) &&
                (KeyBuff[7] == 0x14) &&
                (KeyBuff[8] == 0x46) &&
                (KeyBuff[9] == 0x4D) &&
                (KeyBuff[10] == 0x52)) {
            SRD1 = 0x80;
            SRD2 = 0x0C; // KEY Válida
        }
    }
}
// ************************************************************************** //

void ProgramPage(void) {
    uint16_t addr;

    addr = SRX(); // Lee HIGH address
    addr &= SRX() << 8; // Lee UPPER address

    SRXbuff(RXBUF, 256); // lee los 256 bytes

    WriteBytesFlash(GlobalAddr, 256, RXBUF);
    GlobalAddr += 256;

}
// ************************************************************************** //

void ReadPage(void) {
    uint16_t x;
    uint16_t addr;

    addr = SRX(); // Lee HIGH address
    addr &= SRX() << 8; // Lee UPPER address

    ReadFlash(GlobalAddr2, 256, RXBUF);
    GlobalAddr2 += 256;

    for (x = 0; x < 256; x++) {
        STX(RXBUF[x]);
    }
}
// ************************************************************************** //


