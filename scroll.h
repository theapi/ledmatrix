/*
 * scroll.h
 *
 *  Created on: 31 Mar 2014
 *      Author: peter
 */

#ifndef SCROLL_H_
#define SCROLL_H_

#include <avr/io.h>

#define SC_BUFFER_LEN 64 // Number of characters to remember for scrolling

void scroll_Push(uint8_t c);
uint8_t scroll_Shift();
uint8_t scroll_Empty();
void scroll_Left(uint8_t current[3][8][8], uint8_t next[3][8][8]);
void scroll_LeftMono(uint8_t current[0][8], uint8_t next[0][8]);

#endif /* SCROLL_H_ */
