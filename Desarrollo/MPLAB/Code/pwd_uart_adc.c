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

#include "../../Functions/pwm-v1.h"
#include "../../Functions/uart-v1.h"
#include "../../Functions/adc-v1.h"

int dato, count = 0, canal_actual = 0, conversion_lista = 0;
int lecturas[3];

void init_TMR0(void)
{
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS = 0b110;
    INTCONbits.T0IF = 0;
    INTCONbits.T0IE = 1;
    TMR0 = 60;
}

void __interrupt() interrupt_manager(void)
{    
    if(INTCONbits.T0IF) {
        TMR0 = 60; // 5Mhz Fcy => 5ms / (5ms/5Mhz * 128)

        count++;

        if (count >= 20) {
            ADCON0bits.GO_DONE = 1;
            count = 0;
        } 

        INTCONbits.T0IF = 0;
    }

    if(PIR1bits.ADIF) {
        int valor = (ADRESH << 8) | ADRESL;

        lecturas[canal_actual] = valor;

        canal_actual++;

        if (canal_actual > 2) {
            canal_actual = 0;
            conversion_lista = 1;
        }

        ADCON0bits.CHS = canal_actual;

        PIR1bits.ADIF = 0;
    }
}

void main(void) {
    
    OSCCONbits.OSTS = 1; // External cristal

    init_TMR0();
    
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
   
    init_PWM(20000, 0.3);
    
    uart_init();
    init_ADC();

    while (1) {
        printf("Valores: %d %d %d\n", lecturas[0], lecturas[1], lecturas[2]);
        send_frame(4, 16);
    }
}

