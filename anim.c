/*
 * anim.c
 *
 *  Created on: 10 Apr 2014
 *      Author: peterc
 */

#include "anim.h"

unsigned char anim_source_index[ANIM_SOURCE_LEN]; // Animation key frame index
unsigned char anim_source_time[ANIM_SOURCE_LEN];  // How long to show each key frame
unsigned char anim_head;
unsigned char anim_tail;

/**
 * Push an element onto the end of the scroll buffer.
 */
void anim_Current(uint8_t c)
{
   // Add to the scroll character buffer
   sc_buffer[anim_head] = c;

   // Increment the index
   anim_head++;
   if (anim_head >= ANIM_SOURCE_LEN) {
       anim_head = 0;
   }
}

/**
 * Shift an element off the beginning of the scroll buffer.
 */
uint8_t anim_Next()
{
    uint8_t c = sc_buffer[anim_tail];
    // Increase the processed index
    anim_tail++;
    if (anim_tail >= ANIM_SOURCE_LEN) {
        anim_tail = 0;
    }

    return c;
}

/**
 * Determine whether the anim buffer is empty.
 */
uint8_t anim_Empty()
{
    if (anim_head == anim_tail) {
        return 1;
    }
    return 0;
}
