/*
 * USART.h
 *
 *  Created on: 23 Mar 2014
 *      Author: peter
 */

#ifndef USART_H_
#define USART_H_


#include <avr/interrupt.h>

#define BAUD 57600
#include <util/setbaud.h>

#define RX_BUFFER_LEN 254 // How many bytes the usart receive buffer can hold
#define TX_BUFFER_LEN 254 // How many bytes the usart send buffer can hold

void USART_Init();
void USART_Transmit( unsigned char data );

volatile unsigned char rx_buffer[RX_BUFFER_LEN];
volatile unsigned char rx_head;
unsigned char rx_tail; // The last buffer byte processed

volatile unsigned char tx_buffer[TX_BUFFER_LEN];
volatile unsigned char tx_head;
volatile unsigned char tx_tail;

#endif /* USART_H_ */
