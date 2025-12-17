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

#include "../Functions/pwm-v1.h"
#include "../Functions/uart-v1.h"
#include "../Functions/adc-v1.h"
#include "../Functions/spi-master-v1.h"
#include "../Functions/i2c-v2.h"

#include <stdint.h> // Importante para uint8_t, etc.

#include "../Functions/pwm-v1.h"
#include "../Functions/uart-v1.h"
#include "../Functions/adc-v1.h"
#include "../Functions/spi-master-v1.h"
#include "../Functions/i2c-v2.h"

// --- DEFINICIONES DE SENSORES I2C ---
#define IAQ_ADDR 0x5A
#define VEML7700_ADDR  0x10 
#define VEML_CMD_CONF  0x00 
#define VEML_CMD_ALS   0x04 

// --- VARIABLES GLOBALES ---
volatile int lecturas[3];       // [0]: Ruido, [1]: Humedad, [2]: Temperatura
volatile int canal_actual = 0;
volatile int conversion_lista = 0;

// Variables de sensores I2C
unsigned int co2_prediction = 0;
unsigned int tvoc_prediction = 0;
unsigned char iaq_status = 0;
unsigned long resistance = 0;

// Variables de Actuadores (Desde PC)
float velocidad_pwm = 0.4;
int rojo = 0, verde = 0, azul = 0, intensity = 5;

void IAQ_read(unsigned char* buffer) {
    i2c_start();
    if (i2c_write((IAQ_ADDR << 1) | 1)) { 
        for (int i = 0; i < 8; i++) buffer[i] = i2c_read(1);
        buffer[8] = i2c_read(0);
        i2c_stop();

        // Guardar valores
        co2_prediction = ((unsigned int)buffer[0] << 8) | buffer[1];
        iaq_status = buffer[2];
        resistance = ((unsigned long)buffer[4] << 16) | ((unsigned long)buffer[5] << 8) | buffer[6];
        tvoc_prediction = ((unsigned int)buffer[7] << 8) | buffer[8];
    } else {
        i2c_stop();
    }
}

void VEML_Write(unsigned char command, unsigned int data) {
    i2c_start();
    i2c_write((VEML7700_ADDR << 1) | 0);
    i2c_write(command);
    i2c_write(data & 0xFF);        
    i2c_write((data >> 8) & 0xFF); 
    i2c_stop();
}

unsigned int VEML_Read(unsigned char command) {
    unsigned char lsb, msb;
    unsigned int resultado;
    i2c_start();
    i2c_write((VEML7700_ADDR << 1) | 0);
    i2c_write(command);
    i2c_rstart(); 
    i2c_write((VEML7700_ADDR << 1) | 1);
    lsb = i2c_read(1); 
    msb = i2c_read(0); 
    i2c_stop();
    resultado = ((unsigned int)msb << 8) | lsb;
    return resultado;
}

void VEML_Setup() {
    VEML_Write(VEML_CMD_CONF, 0x0800); // Configuración base
    __delay_ms(200); // Pequeña espera inicial (solo al arranque)
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

void init_TMR0(void)
{
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS = 0b110;
    INTCONbits.T0IF = 0;
    INTCONbits.T0IE = 1;
    TMR0 = 236;
}

void __interrupt() interrupt_manager(void)
{    
    if(INTCONbits.T0IF) {
        TMR0 = 236; // 5Mhz Fcy => 5ms / (5ms/5Mhz * 128)

        ADCON0bits.GO_DONE = 1;

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

void set_leds(int red, int green, int blue, int intensity) {
    //Start frame
    spi_write_read(0x00);
    spi_write_read(0x00);
    spi_write_read(0x00);
    spi_write_read(0x00);

    for (int i = 0 ; i < 10 ; i++) {
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

void Check_UART_RX(void) {
    if(!PIR1bits.RCIF) return; // Si no hay datos, salir inmediatamente

    // Pequeño truco: Si llega un byte, asumimos que llega la trama completa rápido (9600 baud es lento pero continuo)
    // Leemos el cabecera
    uint8_t h = RCREG;
    if(h != 0xAA) return; 

    // Esperar (bloqueo mínimo) al resto de la cabecera. 
    // Al ser polling dentro del if, el impacto es bajo.
    while(!PIR1bits.RCIF); uint8_t len = RCREG;
    while(!PIR1bits.RCIF); uint8_t cmd = RCREG;
    
    uint8_t data_len = len - 1;
    uint8_t buffer[5];
    
    for(int i=0; i<data_len && i<5; i++) {
        while(!PIR1bits.RCIF);
        buffer[i] = RCREG;
    }
    
    // Leer CRC dummy
    while(!PIR1bits.RCIF); RCREG;
    while(!PIR1bits.RCIF); RCREG;

    // --- EJECUTAR COMANDO RECIBIDO ---
    if (cmd == 0x04 && data_len >= 1) { // Ventilador
        int val = buffer[0];
        if(val > 100) val = 100;
        velocidad_pwm = (float)val / 100.0;
        set_PWM(20000, velocidad_pwm); // Actualizar PWM inmediatamente
    }
    else if (cmd == 0x05 && data_len >= 4) { // LEDs
        rojo = buffer[0];
        verde = buffer[1];
        azul = buffer[2];
        intensity = buffer[3] & 0x1F;
        set_leds(rojo, verde, azul, intensity); // Actualizar LEDs inmediatamente
    }
}

void main(void) {
    unsigned char buffer[9];
    uint8_t payload[4];
    unsigned int luz_raw;
    float lux_value;
    int count = 0;

    OSCCONbits.OSTS = 1; 

    // Inicializaciones
    init_ADC();
    uart_init();     // 9600 baud
    i2c_init_setup();
    init_spi();
    init_PWM(20000, velocidad_pwm);
    
    // Configurar interrupciones al final
    init_TMR0();
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;

    // Setup inicial de sensores lentos
    VEML_Setup();
    set_leds(0, 0, 255, 5);
   
    while(1){
        // TRUCO: En vez de dormir 500ms del tirón, dormimos 10ms 50 veces.
        // Así podemos revisar si el PC nos habla constantemente.
        for(int k=0; k<50; k++) {
            __delay_ms(10);
            Check_UART_RX(); // <--- DESCOMENTADO Y VITAL
        }
        
        count++;
        
        if (count == 4) {
            IAQ_read(buffer);

            luz_raw = VEML_Read(VEML_CMD_ALS);
            lux_value =  (float)luz_raw * 0.042;
            
            // --- 1. LUMINOSIDAD (Comando 0x01, 2 bytes) ---
            unsigned int lux_int = (unsigned int)lux_value; 
            payload[0] = (lux_int >> 8) & 0xFF; // MSB
            payload[1] = (lux_int) & 0xFF;      // LSB
            send_frame(0x01, 2, payload);

            // --- 2. CO2 (Comando 0x02, 2 bytes) ---
            payload[0] = (co2_prediction >> 8) & 0xFF;
            payload[1] = (co2_prediction) & 0xFF;
            send_frame(0x02, 2, payload);

            // --- 3. HUMEDAD (Comando 0x03, 2 bytes) ---
            unsigned int hum_int = (unsigned int)lecturas[1];
            payload[0] = (hum_int >> 8) & 0xFF;
            payload[1] = (hum_int) & 0xFF;
            send_frame(0x03, 2, payload);

            // --- 4. VENTILADOR (Comando 0x04, 1 byte) ---
            uint8_t fan_percent = (uint8_t)(velocidad_pwm * 100); 
            payload[0] = fan_percent;
            send_frame(0x04, 1, payload);

            // --- 5. LEDS (Comando 0x05, 4 bytes) ---
            payload[0] = (uint8_t)rojo;
            payload[1] = (uint8_t)verde;
            payload[2] = (uint8_t)azul;
            payload[3] = (uint8_t)intensity;
            send_frame(0x05, 4, payload);

            // --- 6. TEMPERATURA (Comando 0x06, 1 byte) ---
            payload[0] = (uint8_t)lecturas[2];
            send_frame(0x06, 1, payload);
            
            count = 0;
        }
        
        if (count == 1) {
            uint8_t noise_category = 0;
            int adc_noise = lecturas[0];
            
            if (adc_noise <= 400) {
                noise_category = 0; // Bajo
            } else if (adc_noise <= 900) {
                noise_category = 1; // Intermedio
            } else {
                noise_category = 2; // Alto
            }
            
            payload[0] = noise_category;
            send_frame(0x00, 1, payload);
        }
        
        //printf("Luz: %i CO2: %i\r\nValores: %i %i %i\r\n", lux_value, co2_prediction, lecturas[0], lecturas[1], lecturas[2]);
    }
}