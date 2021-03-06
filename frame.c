/*
 * frame.c
 *
 *  Created on: 23 Mar 2014
 *      Author: peter
 */

#include "frame.h"

/**
 * Sets a coloured pixel in the given frame.
 */
void frame_SetPixel(uint8_t frame[3][8][8], uint8_t row, uint8_t col, uint8_t r, uint8_t g, uint8_t b)
{
    frame[0][row][col] = r;
    frame[1][row][col] = g;
    frame[2][row][col] = b;
}

void frame_ColourFill(uint8_t frame[3][8][8], uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t row;
    uint8_t col;

    for (row = 0; row < 8; row++) {
        for (col = 0; col < 8; col++) {
            frame[0][row][col] = r;
            frame[1][row][col] = g;
            frame[2][row][col] = b;
        }
    }
}

/**
 * Adds colour to a PROGMEM mono frame (e.g. font).
 * A mono frame simply has bits to say an led is on, a colour frame has uint8_t values instead of bits.
 */
void frame_Colourise_P(uint8_t frame[3][8][8], uint8_t mono[8], uint8_t colours[3][8][8])
{
    uint8_t row;
    uint8_t col;

    for (row = 0; row < 8; row++) {
        uint8_t bit = 7;
        for (col = 0; col < 8; col++) {

            uint8_t byte = pgm_read_byte(mono + row);

            // Check whether the col bit is set in the mono frame.
            if (byte & (1 << bit)) {
                // Use the equivelent colour for the pixel.
                frame[0][row][col] = colours[0][row][col];
                frame[1][row][col] = colours[1][row][col];
                frame[2][row][col] = colours[2][row][col];
            } else {
                // Pixel off
                frame[0][row][col] = 0x00;
                frame[1][row][col] = 0x00;
                frame[2][row][col] = 0x00;
            }

            --bit;
        }
    }
}

/**
 * Stores the data for the frames so the bytes are uint8_t values of the brightness.
 */
void frame_SetColoured(uint8_t frame[3][8][8], uint8_t red[8][8], uint8_t green[8][8], uint8_t blue[8][8])
{
    uint8_t row;
    uint8_t col;

    for (row = 0; row < 8; row++) {
        for (col = 0; col < 8; col++) {
            frame[0][row][col] = red[row][col];
            frame[1][row][col] = green[row][col];
            frame[2][row][col] = blue[row][col];
        }
    }
}

/**
 * Stores the data for the frame, one byte per row.
 */
void frame_SetMono(uint8_t frame[0][8], uint8_t red[8], uint8_t green[8], uint8_t blue[8])
{
  uint8_t i;
  for (i = 0; i < 8; i++) {
      frame[0][i] = ~red[i];
      frame[1][i] = ~green[i];
      frame[2][i] = ~blue[i];
  }
}

/**
 * Stores the data from PROGMEM for the frame, one byte per row.
 */
void frame_SetMono_P(uint8_t frame[0][8], uint8_t red[8], uint8_t green[8], uint8_t blue[8])
{
  uint8_t i;
  for (i = 0; i < 8; i++) {
      frame[0][i] = ~pgm_read_byte(red + i);
      frame[1][i] = ~pgm_read_byte(green + i);
      frame[2][i] = ~pgm_read_byte(blue + i);
  }
}


void frame_Rotate(uint8_t frame[0][8], int degrees)
{
    uint8_t tmp[3][8];
    uint8_t i;
    uint8_t j;

    for (i = 0; i <= 7; i++){
        tmp[0][i] = frame[0][i];
        tmp[1][i] = frame[1][i];
        tmp[2][i] = frame[2][i];
    }

    if (degrees == 270) {
      for (i = 0; i <= 7; i++){
        for (j = 0; j <= 7; j++){
            bitWrite(frame[0][i], j, bitRead(tmp[0][7 - j], (i)) );
            bitWrite(frame[1][i], j, bitRead(tmp[1][7 - j], (i)) );
            bitWrite(frame[2][i], j, bitRead(tmp[2][7 - j], (i)) );
        }
      }
    }
}

/**
 * Flip the frame on the horizontal axis.
 */
void frame_FlipH(uint8_t frame[0][8])
{
    uint8_t i;

    for (i = 0; i < 3; i++) {
        frame_SwapBytes(&frame[i][0], &frame[i][7]);
        frame_SwapBytes(&frame[i][1], &frame[i][6]);
        frame_SwapBytes(&frame[i][2], &frame[i][5]);
        frame_SwapBytes(&frame[i][3], &frame[i][4]);
    }
}

/**
 * Flip the frame on the vertical axis.
 */
void frame_FlipV(uint8_t frame[0][8])
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        // red
        frame[0][i] = frame_BitReverse(frame[0][i]);
        // green
        frame[1][i] = frame_BitReverse(frame[1][i]);
        // blue
        frame[2][i] = frame_BitReverse(frame[2][i]);
    }

}

void frame_SwapBytes(uint8_t *px, uint8_t *py)
{
    uint8_t temp;

    temp = *px;
    *px = *py;
    *py = temp;
}

/**
 * Make least significant bit highest etc.
 *
 * 11001100 -> 00110011
 */
uint8_t frame_BitReverse(uint8_t x)
{
/*
    uint8_t i;
    uint8_t reversed = 0;

    for (i = 0; i < 8; i++)  {
        reversed >>= 1;
        reversed |= (x & (1 << 7));
        x <<= 1;
    }

    return reversed;
*/

    /*

    The following is faster than a loop, taken from from USI_UART.c AVR307.

    So I can see the bitwise operators in action:

    For x = 11001100...

    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 1) & 01010101) | ((x << 1) & 10101010);
    x = (01100110 & 01010101) | (10011000 & 10101010);
    x = 01000100 | 10001000;
    x = 11001100;


    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 2) & 00110011) | ((x << 2) & 11001100);
    x = (00110011 & 00110011) | (00110000 & 11001100);
    x = 00110011 | 00000000;
    x = 00110011;


    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    x = ((x >> 4) & 00001111) | ((x << 4) & 11110000);
    x = (00000011 & 00001111) | (00110000 & 11110000);
    x = 00000011 | 00110000;
    x = 00110011;

    */

    // from USI_UART.c AVR307
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;
}

