/*
 * matrix.h
 *
 *  Created on: 23 Mar 2014
 *      Author: peter
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#define PIN_DATA   PB3 // DS - MOSI - 11
#define PIN_CLOCK  PB5 // SH_CP - SCK - 13
#define PIN_LATCH  PB1 // ST_CP - 9
#define PIN_OE     PC3 // Output Enable - 17

#define MAX_CYCLE_COUNT 16 // number of cycles needed brightness (a cycle is lighting each row once)

// From Android.h
#define LSBFIRST 0
#define MSBFIRST 1

void latchLow(void);
void latchHigh(void);
void ledsDisable(void);
void ledsEnable(void);
void sendLine(uint8_t red, uint8_t blue, uint8_t green);
void sendByte(uint8_t bitOrder, uint8_t byte);



#endif /* MATRIX_H_ */
