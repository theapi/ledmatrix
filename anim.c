/*
 * anim.c
 *
 *  Created on: 10 Apr 2014
 *      Author: peterc
 */

#include "anim.h"

uint8_t anim_source_index[ANIM_SOURCE_LEN]; // Animation key frame index
unsigned long anim_source_duration[ANIM_SOURCE_LEN];  // How long to show each key frame
uint8_t anim_head;
uint8_t anim_tail;
uint8_t anim_length; // How long the anim is.
uint8_t anim_current;
uint8_t anim_begin;


/**
 * Add to the animation.
 */
void anim_Add(uint8_t source_index, unsigned long duration)
{
   // Add to the scroll character buffer
   anim_source_index[anim_head] = source_index;
   anim_source_duration[anim_head] = duration; // millis

   // Increment the index
   anim_head++;
   if (anim_head >= ANIM_SOURCE_LEN) {
       anim_head = 0;
   }
}

void anim_SetLength(uint8_t length)
{
    anim_length = length;
}

void anim_SetBeginning()
{
    anim_begin = anim_head;
}

/**
 * How long the current index should be shown in milliseconds.
 */
uint8_t anim_CurrentIndex()
{
    return anim_source_index[anim_tail];
}

/**
 * How long the current index should be shown in milliseconds.
 */
unsigned long anim_CurrentDuration()
{
    return anim_source_duration[anim_tail];
}

/**
 * Move on the the next index.
 */
void anim_Next()
{
    anim_tail++;
    if (anim_tail >= ANIM_SOURCE_LEN) {
        anim_tail = 0;
    }

    anim_current++;
    if (anim_current > anim_length) {
        anim_current = 0;
        anim_tail = anim_begin;
    }

    /*
    if (anim_current >= anim_length) {
        anim_current = 0;
        anim_tail = anim_begin;
    } else {
        anim_current++;
        // Increase the processed index
        anim_tail++;
        if (anim_tail >= ANIM_SOURCE_LEN) {
            anim_tail = 0;
        }
    }
    */
}

/**
 * Determine whether the anim buffer is empty.
 */
uint8_t anim_Complete()
{
    if (anim_head == anim_tail) {
        return 1;
    }
    return 0;
}
