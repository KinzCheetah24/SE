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

#include "../../Functions/i2c-v2.h"

// Dirección I2C: 0x5A (90 decimal)
#define IAQ_ADDR 0x5A

// Dirección del esclavo (7 bits = 0x10) [cite: 804]
// Para escritura (RW=0) -> 0x20
// Para lectura  (RW=1) -> 0x21
#define VEML7700_ADDR  0x10 

// Comandos de registro [cite: 821]
#define VEML_CMD_CONF  0x00 // Registro de configuración
#define VEML_CMD_ALS   0x04 // Registro de lectura de luz ambiental

// Variables para almacenar los resultados combinados
unsigned int co2_prediction = 0;
unsigned int tvoc_prediction = 0;
unsigned char status = 0;
unsigned long resistance = 0;

void IAQ_read(unsigned char* buffer) {
    i2c_start();
        
        // Dirección 0x5A desplazada + bit de lectura (1) = 0xB5 [cite: 93]
        // i2c_write retorna 1 si hubo ACK, 0 si hubo NACK.
        if (i2c_write((IAQ_ADDR << 1) | 1)) { 
            
            // Leer Bytes 0 a 7 con ACK (master_ack = 1)
            for (int i = 0; i < 8; i++) {
                buffer[i] = i2c_read(1); // 1 = ACK
            }
            
            // Leer el último Byte (8) con NACK (master_ack = 0)
            // El maestro debe enviar NACK al final 
            buffer[8] = i2c_read(0); // 0 = NACK
            
            i2c_stop();

            // --- Procesamiento de Datos ---
            
            // 1. Predicción CO2 (Bytes 0 y 1) [cite: 353]
            // Fórmula: (Byte0 * 256) + Byte1
            co2_prediction = ((unsigned int)buffer[0] << 8) | buffer[1];

            // 2. Estado (Byte 2) [cite: 109]
            // 0x00: OK, 0x10: Calentando, 0x01: Ocupado, 0x80: Error
            status = buffer[2];

            // 3. Resistencia (Bytes 4, 5 y 6) - Byte 3 es siempre 0 [cite: 370]
            resistance = ((unsigned long)buffer[4] << 16) | 
                         ((unsigned long)buffer[5] << 8) | 
                         buffer[6];

            // 4. TVOC (Bytes 7 y 8) 
            tvoc_prediction = ((unsigned int)buffer[7] << 8) | buffer[8];
            
        } else {
            // El sensor no respondió (NACK en la dirección)
            i2c_stop();
        }
}

void i2c_init_setup(void) {
    // Configuración básica para I2C Maestro a 100kHz (ajustar según tu XTAL)
    TRISCbits.TRISC3 = 1; // SCL input (el módulo lo controla)
    TRISCbits.TRISC4 = 1; // SDA input
    SSPCON = 0b00101000;  // I2C Master mode, Enable
    SSPADD = 49;          // Baud Rate para 100kHz con Fosc=20MHz ((20M/4/100k)-1)
    SSPSTAT = 0;
    
    // 4. Limpiar estado y bandera
    SSPSTAT = 0x00; // Slew rate control disabled (standard speed)
    PIR1bits.SSPIF = 0;
}

// Función para escribir 16 bits en un registro del VEML7700
void VEML_Write(unsigned char command, unsigned int data) {
    i2c_start();
    i2c_write((VEML7700_ADDR << 1) | 0); // Dirección 0x20 (Write)
    i2c_write(command);                  // Registro al que queremos escribir
    
    // El datasheet especifica enviar primero el LSB y luego el MSB [cite: 775, 777]
    i2c_write(data & 0xFF);        // LSB
    i2c_write((data >> 8) & 0xFF); // MSB
    
    i2c_stop();
}

// Función para leer 16 bits de un registro
unsigned int VEML_Read(unsigned char command) {
    unsigned char lsb, msb;
    unsigned int resultado;

    // Paso 1: Indicar qué registro queremos leer (Write operation)
    i2c_start();
    i2c_write((VEML7700_ADDR << 1) | 0); // 0x20 Write
    i2c_write(command);
    
    // Paso 2: Reiniciar comunicación para leer (Repeated Start)
    i2c_rstart(); 
    i2c_write((VEML7700_ADDR << 1) | 1); // 0x21 Read

    // Paso 3: Leer los dos bytes
    lsb = i2c_read(1); // ACK (queremos leer otro byte después de este)
    msb = i2c_read(0); // NACK (es el último byte, cerramos comunicación) [cite: 787]
    
    i2c_stop();

    // Reconstruir el valor de 16 bits
    resultado = ((unsigned int)msb << 8) | lsb;
    return resultado;
}

// Configuración inicial del sensor
void VEML_Setup() {
    // Configuración básica (Registro 0x00):
    // ALS_SD (Bit 0) = 0 (Encender sensor) [cite: 821]
    // ALS_IT (Bits 9:6) = 0000 (100ms integración)
    // ALS_SM (Bits 12:11) = 01 (Ganancia x2)
    // Bits 12:11 = 01 -> 0x0800
    // El resto en 0. Valor total a escribir = 0x0800.
    
    VEML_Write(VEML_CMD_CONF, 0x0800);
    
    // Importante: Esperar un poco tras encender para que haga la primera medida
    // Según tabla de refresco, para IT=100ms, esperar min 600ms es seguro [cite: 893]
    __delay_ms(600);
}

void main(void) {
    unsigned char buffer[9]; // Buffer para los 9 bytes del sensor 
    unsigned int luz_raw;
    float lux_value;
    
    OSCCONbits.OSTS = 1; 
    i2c_init_setup(); // IMPORTANTE: Inicializar el periférico
    VEML_Setup();     // Encender y configurar el sensor de luz
    
    while (1) {
        IAQ_read(buffer);
        
        luz_raw = VEML_Read(VEML_CMD_ALS);
        lux_value =  (float)luz_raw * 0.042;

        __delay_ms(1000); // Esperar antes de la siguiente lectura
        // El intervalo de medición en modo continuo es 1s [cite: 47]
    }
}

