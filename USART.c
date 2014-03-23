/*
 * USART.c
 *
 *  Created on: 23 Mar 2014
 *      Author: peter
 */

#include "USART.h"

volatile unsigned char rx_buffer[RX_BUFFER_LEN];
volatile unsigned char rx_head;
unsigned char rx_tail; // The last buffer byte processed

volatile unsigned char tx_buffer[TX_BUFFER_LEN];
volatile unsigned char tx_head;
volatile unsigned char tx_tail;

/**
 * Interrupt when the USART receives a byte.
 */
ISR(USART_RX_vect)
{
   // Add to the receive buffer
   rx_buffer[rx_head] = UDR0;

   // Increment the index
   rx_head++;
   if (rx_head >= RX_BUFFER_LEN) {
       rx_head = 0;
   }
}

/**
 * Interrupt when the USART is ready to send more bytes.
 */
ISR(USART_UDRE_vect)
{
    // If head & tail are not in sync, send the next byte in byte.
    if (tx_head != tx_tail) {
        UDR0 = tx_buffer[tx_tail];
        // Increment the tail index
        tx_tail++;
        if (tx_tail >= TX_BUFFER_LEN) {
            tx_tail = 0;
        }
    } else {
        // Nothing left to send so turn off this interrupt
        UCSR0B &= ~(1 << UDRIE0);
    }
}


void USART_Init(void)
{
    //UBRR0H = (UBRR >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
    //UBRR0L = UBRR;        // Load lower 8-bits of the baud rate value into the low byte of the UBRR register

    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    #if USE_2X
        UCSR0A |= (1 << U2X0);
    #else
        UCSR0A &= ~(1 << U2X0);
    #endif

    // Use 8-bit character sizes & 1 stop bit
    UCSR0C = (0 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01);

    // Enable receiver, transmitter and interrupt on receive.
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
}

void USART_Transmit( unsigned char data )
{
    // Buffer the byte to be sent by the ISR
    tx_buffer[tx_head] = data;
    // Increment the head index
    tx_head++;
    if (tx_head >= TX_BUFFER_LEN) {
        tx_head = 0;
    }

    // Ensure the interrupt to send this is on.
    UCSR0B |= (1 << UDRIE0);

    /*
    // Wait for empty transmit buffer
    while ( !( UCSR0A & (1 << UDRE0)) )
        ;
    // Put data into buffer, sends the data
    UDR0 = data;
    */
}

uint8_t USART_Empty()
{
    if (rx_head == rx_tail) {
        return 1;
    }
    return 0;
}

uint8_t USART_ReadByte()
{
    uint8_t c = rx_buffer[rx_tail];
    // Increase the processed index
    rx_tail++;
    if (rx_tail >= RX_BUFFER_LEN) {
        rx_tail = 0;
    }

    return c;
}

