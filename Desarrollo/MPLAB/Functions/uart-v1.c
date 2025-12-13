/* 
 * File:   uart-v1.c
 * Author: kinz
 *
 * Created on December 7, 2025, 1:54 PM
 */

#include <xc.h>

void uart_init(void)
{  
    TXSTAbits.BRGH = 0;
    BAUDCTLbits.BRG16 = 0;

    // SPBRGH:SPBRG = 
    SPBRGH = 0;
    SPBRG = 32;  // 9600 baud rate with 20MHz Clock

    TXSTAbits.SYNC = 0; /* Asynchronous */
    TXSTAbits.TX9 = 0; /* TX 8 data bit */
    RCSTAbits.RX9 = 0; /* RX 8 data bit */

    PIE1bits.TXIE = 0; /* Disable TX interrupt */
    PIE1bits.RCIE = 0; /* Disable RX interrupt */

    RCSTAbits.SPEN = 1; /* Serial port enable */

    TXSTAbits.TXEN = 1; /* Enable transmitter */
    RCSTAbits.CREN = 1; /* Enable reception */
  
}

/* It is needed for printf */
void putch(char c)
{ 
    while(!TXIF);
    TXREG = c;
}

/* ------------------------------
   Función de recepción bloqueante
   ------------------------------ */
char uart_read(void)
{
    while(!PIR1bits.RCIF);   // Espera a que llegue un byte

    // Manejo opcional de errores:
    if(RCSTAbits.OERR){      // Overrun error
        RCSTAbits.CREN = 0;  // reset
        RCSTAbits.CREN = 1;
    }

    return RCREG;            // Devuelve el byte recibido
}