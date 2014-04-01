/*
 * scroll.c
 *
 *  Created on: 31 Mar 2014
 *      Author: peter
 */

#include "scroll.h"

unsigned char sc_buffer[SC_BUFFER_LEN]; // Scroll character buffer
unsigned char sc_head;
unsigned char sc_tail;

/**
 * Push an element onto the end of the scroll buffer.
 */
void scroll_Push(uint8_t c)
{
   // Add to the scroll character buffer
   sc_buffer[sc_head] = c;

   // Increment the index
   sc_head++;
   if (sc_head >= SC_BUFFER_LEN) {
       sc_head = 0;
   }
}

/**
 * Shift an element off the beginning of the scroll buffer.
 */
uint8_t scroll_Shift()
{
    uint8_t c = sc_buffer[sc_tail];
    // Increase the processed index
    sc_tail++;
    if (sc_tail >= SC_BUFFER_LEN) {
        sc_tail = 0;
    }

    return c;
}

/**
 * Determine whether the scroll buffer is empty.
 */
uint8_t scroll_Empty()
{
    if (sc_head == sc_tail) {
        return 1;
    }
    return 0;
}


