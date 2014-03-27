/*
 * USART.h
 *
 *  Created on: 23 Mar 2014
 *      Author: peter
 */

#ifndef USART_H_
#define USART_H_


#define BAUD 115200

#include <util/setbaud.h>
#include <avr/interrupt.h>

#define RX_BUFFER_LEN 254 // How many bytes the usart receive buffer can hold
#define TX_BUFFER_LEN 254 // How many bytes the usart send buffer can hold

void USART_Init();
void USART_Transmit( unsigned char data );
uint8_t USART_Empty();
uint8_t USART_ReadByte();

#endif /* USART_H_ */
