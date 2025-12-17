#include <xc.h>
#include <stdio.h>

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
    PIE1bits.RCIE = 0; /* Enable RX interrupt */ //CAMBIADO

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
    // 1. Cabecera 0xAA (No "170")
    uart_write(0xAA);
    
    // 2. Longitud
    uart_write(length + 1);
    
    // 3. Comando
    uart_write(command);
    
    // 4. Datos
    for (int i = 0 ; i < length ; i++) {
        uart_write(data[i]);
    }
    
    // 5. CRC (2 bytes) - Sin \r\n
    uart_write(0x00);
    uart_write(0x00);
    
    /*
    printf("%u%u%u",0xAA, length + 1, command);
    
    for (int i = 0 ; i < length ; i++) {
        printf("%u", data[i]);
    }
    
    printf("%u%u%\r\n", 0x00,0x00);
    */
}