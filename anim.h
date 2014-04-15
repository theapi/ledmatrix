/*
 * anim.h
 *
 *  Created on: 10 Apr 2014
 *      Author: peterc
 */

#ifndef ANIM_H_
#define ANIM_H_

#include <avr/io.h>

#define ANIM_SOURCE_LEN 9 // The max number of key frames in the animation

void anim_Add(uint8_t source_index, unsigned long duration);
uint8_t anim_CurrentIndex();
unsigned long anim_CurrentDuration();
void anim_Next();
uint8_t anim_Complete();
void anim_SetBeginning();
void anim_SetLength(uint8_t length);

#endif /* ANIM_H_ */
