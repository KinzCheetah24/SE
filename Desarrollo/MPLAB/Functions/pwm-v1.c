/* 
 * File:   pwm-v1.c
 * Author: kinz
 *
 * Created on December 7, 2025, 14:14 PM
 */

#include <xc.h>

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 20000000
#endif

void init_PWM(int freq, float porcentaje)
{
    PR2 = 5000000 / (4 * freq * 1) - 1; // PR2 => 5Mhz / (4 * 25Khz * 1)
    T2CONbits.T2CKPS = 0b00;
    T2CONbits.TMR2ON = 1;
    
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC1 = 0;
    CCP1CONbits.CCP1M = 0b1100;
    CCPR1L = PR2 * porcentaje;
}
