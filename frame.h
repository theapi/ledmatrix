/*
 * frame.h
 *
 *  Created on: 23 Mar 2014
 *      Author: peter
 */

#ifndef FRAME_H_
#define FRAME_H_

#include <avr/io.h>

void frame_setColoured(uint8_t frame[3][8][8], uint8_t red[8][8], uint8_t green[8][8], uint8_t blue[8][8]);

#endif /* FRAME_H_ */
