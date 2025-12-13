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
    PIE1bits.RCIE = 1; /* Enable RX interrupt */

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
uint16_t uart_read(void)
{
    while(!PIR1bits.RCIF);   // Espera a que llegue un byte

    // Manejo opcional de errores:
    if(RCSTAbits.OERR){      // Overrun error
        RCSTAbits.CREN = 0;  // reset
        RCSTAbits.CREN = 1;
    }

    return RCREG;            // Devuelve el byte recibido
}

void uart_write(uint16_t c) {
    while(!TXIF); // Esperar a que el buffer esté libre
    TXREG = c;
}

void send_frame(uint8_t command, uint8_t length, uint8_t *data) {
    uart_write(0xAA);           // Header
    uart_write(length);           // Length (1 byte comando + 2 bytes payload)
    uart_write(command);        // Command
    
    for (int i = 0 ; i < length ; i++) {
        uart_write(data[i]);
    }
    
    uart_write(0x00);           // CRC 1 (Dummy)
    uart_write(0x00);           // CRC 2 (Dummy)
}