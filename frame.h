/*
 * frame.h
 *
 *  Created on: 23 Mar 2014
 *      Author: peter
 */

#ifndef FRAME_H_
#define FRAME_H_

#include <avr/io.h>
#include <avr/pgmspace.h>

// From Android.h
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))


void frame_SetColoured(uint8_t frame[3][8][8], uint8_t red[8][8], uint8_t green[8][8], uint8_t blue[8][8]);
void frame_SetMono(uint8_t frame[0][8], const uint8_t red[8], const uint8_t green[8], const uint8_t blue[8]);
void frame_SetMono_P(uint8_t frame[0][8], const uint8_t red[8], const uint8_t green[8], const uint8_t blue[8]);
void frame_Rotate(uint8_t frame[0][8], int degrees);
void frame_FlipH(uint8_t frame[0][8]);
void frame_FlipV(uint8_t frame[0][8]);
void frame_SwapBytes(uint8_t *px, uint8_t *py);
uint8_t frame_BitReverse(uint8_t x);


#endif /* FRAME_H_ */
