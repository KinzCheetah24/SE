/* 
 * File:   adc-v1.c
 * Author: kinz
 *
 * Created on December 7, 2025, 15:12 PM
 */

#include <xc.h>

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 20000000
#endif

void init_ADC(void) {
    ANSEL  = 0b00000111;    // AN0, AN1, AN2 enabled as analog
    ANSELH = 0;             // no analog on higher pins
    
    TRISAbits.TRISA0 = 1;   // RA0 entrada
    TRISAbits.TRISA1 = 1;   // RA1 entrada
    TRISAbits.TRISA2 = 1;
    
    ADCON0bits.CHS = 0b00;
    ADCON0bits.ADCS = 0b10;

    ADCON0bits.ADON = 1;

    ADCON1bits.ADFM = 1;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;

    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
}