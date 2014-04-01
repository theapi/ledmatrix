/*
 * scroll.h
 *
 *  Created on: 31 Mar 2014
 *      Author: peter
 */

#ifndef SCROLL_H_
#define SCROLL_H_

#include <avr/io.h>

#define SC_BUFFER_LEN 32 // Number of characters to remember for scrolling

void scroll_Push(uint8_t c);
uint8_t scroll_Shift();
uint8_t scroll_Empty();

#endif /* SCROLL_H_ */
