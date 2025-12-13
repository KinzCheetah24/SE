/* 
 * File:   uart-v1.h
 * Author: kinz
 *
 * Created on December 7, 2025, 1:54 PM
 */

#ifndef UART_V1_H
#define	UART_V1_H

void uart_init(void);

uint16_t uart_read(void);

void uart_write(uint16_t c);

void send_frame(uint8_t command, uint8_t length, uint8_t *data);

#endif	/* UART_V1_H */
