/*
 * matrix.c
 *
 *  Created on: 23 Mar 2014
 *      Author: peter
 */

#include <avr/io.h>
#include "matrix.h"


// anode low = ON with pnp
uint8_t anodes = 0b01111111;


uint8_t junk; // throwaway variable for shift register SPI return data

/**
 * Set the latch low.
 */
void latchLow(void)
{
    PORTB &= ~(1 << PIN_LATCH);
}

/**
 * Take the latch pin high, to move the shifted data into position
 */
void latchHigh(void)
{
    PORTB |= (1 << PIN_LATCH);
}

/**
 * OE low to turn on the leds.
 */
void ledsEnable(void)
{
    PORTC &= ~(1 << PIN_OE);
}

/**
 * Turn off the leds with OE high.
 *
 * To prevent ghosting/leakage.
 * The process seems to need time to settle (parasitic capcitance?)
 */
void ledsDisable(void)
{
    PORTC |= (1 << PIN_OE);
}

/**
 * Send out the next line of prepared data.
 */
void sendLine(uint8_t red, uint8_t green, uint8_t blue)
{

    //if (cycle_count < 5) {
        sendByte(MSBFIRST, red);
    //} else {
      //  sendByte(MSBFIRST, ~0);
    //}

    //if (cycle_count < 12) {
        sendByte(MSBFIRST, blue);
    //} else {
      //  sendByte(MSBFIRST, ~0);
    //}

    // YEP, the hardware I built needs the green to be least significate bit first :(
    //if (cycle_count < 16) {
        sendByte(LSBFIRST, green);
    //} else {
      //  sendByte(LSBFIRST, ~0);
    //}

    // ANODES
    sendByte(MSBFIRST, anodes);

    // Prepare the anodes for the next call.
    // see http://en.wikipedia.org/wiki/Circular_shift
    anodes = (anodes >> 1) | (anodes << 7);


}

void sendByte(uint8_t bitOrder, uint8_t byte)
{
/*
    // Bit bang
    uint8_t i;
    uint8_t bit;

    for (i = 0; i < 8; i++)  {
        if (bitOrder == LSBFIRST) {
            bit = i;
        } else {
            bit = 7 - i;
        }

        if (byte & (1 << bit)) {
            PORTB |= (1 << PIN_DATA); // HIGH
        } else {
            PORTB &= ~(1 << PIN_DATA); // LOW
        }

        // Clock in the bit
        PORTB |= (1 << PIN_CLOCK); // HIGH
        PORTB &= ~(1 << PIN_CLOCK); // LOW
    }
*/

    // SPI
    if (bitOrder == LSBFIRST) {
        SPCR |= (1 << DORD); // high
    } else {
        SPCR &= ~(1 << DORD); // low
    }

    // Send the byte
    SPDR = byte;
    // Wait for SPI process to finish
    while(!(SPSR & (1<<SPIF)));
    // Need to read the result
    junk = SPDR;

}

