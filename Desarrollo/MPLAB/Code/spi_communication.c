// PIC16F886 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR21V   // Brown-out Reset Selection bit (Brown-out Reset set to 2.1V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF

#pragma intrinsic(_delay)
#define _XTAL_FREQ 20000000 // necessary for __delay_us

#include <xc.h>

#include <stdio.h>
#include <stdlib.h>

#include "../../Functions/spi-master-v1.h"

void set_leds(int red, int green, int blue, int intensity) {
    //Start frame
    spi_write_read(0x00);
    spi_write_read(0x00);
    spi_write_read(0x00);
    spi_write_read(0x00);

    for (int i = 0 ; i < 8 ; i++) {
        //LED frame
        spi_write_read(0b11100000 | intensity);  // 111 + 5 bytes brightness adjustment
        spi_write_read(0x00 | blue);  // BLUE level
        spi_write_read(0x00 | green);  // GREEN level
        spi_write_read(0x00 | red);  // RED level
    }

    //END Frame
    spi_write_read(0xFF);
    spi_write_read(0xFF);
    spi_write_read(0xFF);
    spi_write_read(0xFF);
}

void main(void) {
    while(1){
        set_leds(0, 0, 0);
    }
}

