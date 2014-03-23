/*
 * frame.c
 *
 *  Created on: 23 Mar 2014
 *      Author: peter
 */

#include "frame.h"

/**
 * ...
 */
void frame_setColoured(uint8_t frame[3][8][8], uint8_t red[8][8], uint8_t green[8][8], uint8_t blue[8][8])
{
  uint8_t i;
  uint8_t j;

  for (i = 0; i < 8; i++) {
      for (j = 0; j < 8; j++) {
        frame[0][i][j] = red[i][j];
        frame[1][i][j] = green[i][j];
        frame[2][i][j] = blue[i][j];
      }
  }
}
