/*
 * scroll.c
 *
 *  Created on: 31 Mar 2014
 *      Author: peter
 */

#include "scroll.h"
#include "frame.h"

unsigned char sc_buffer[SC_BUFFER_LEN]; // Scroll character buffer
unsigned char sc_head;
unsigned char sc_tail;

void scroll_LeftMono(uint8_t current[0][8], uint8_t next[0][8])
{
      uint8_t i;
      for (i = 0; i < 8; i++) {
          // Shift the current frame one pixel left.
          current[0][i] <<= 1;
          current[1][i] <<= 1;
          current[2][i] <<= 1;

          // Replace the rightmost bits with the leftmost bits of the next frame.
          bitWrite(current[0][i], 0, bitRead(next[0][i], 7));
          bitWrite(current[1][i], 0, bitRead(next[1][i], 7));
          bitWrite(current[2][i], 0, bitRead(next[2][i], 7));

          // Shift the next frame ready for the next round.
          next[0][i] <<= 1;
          next[1][i] <<= 1;
          next[2][i] <<= 1;
      }
}

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


