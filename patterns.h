/*
 * patterns.h
 *
 *  Created on: 24 Mar 2014
 *      Author: peterc
 */

#ifndef PATTERNS_H_
#define PATTERNS_H_

uint8_t patterns[SOURCE_SIZE_PATTERNS][8] PROGMEM = {
    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFF}, // |_
    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFF, 0xFF}, // |_
    {0x80, 0x80, 0x80, 0x80, 0x80, 0xFF, 0x80, 0xFF}, // |_
    {0x00, 0x66, 0x66, 0x00, 0x42, 0x42, 0x3C, 0x00}, // :)
    {0x00, 0x06, 0x66, 0x00, 0x42, 0x42, 0x3C, 0x00}, // ;)
    {0x00, 0x00, 0x66, 0x00, 0x42, 0x42, 0x3C, 0x00}, // |)
    {0x00, 0x66, 0x66, 0x00, 0x7E, 0x7E, 0x3C, 0x00}, // :O
};

/*
uint8_t image[3][8][8] =
{
    { // red
        {13, 13, 13, 13, 13, 14, 14, 15 },
        {13, 12, 11, 11, 13, 13, 2, 2 },
        {13, 11, 11, 12, 13, 13, 13, 13 },
        {13, 11, 11, 12, 13, 13, 13, 13 },
        {14, 13, 13, 13, 13, 12, 2, 2 },
        {13, 13, 13, 13, 13, 12, 2, 2 },
        {13, 13, 13, 13, 13, 12, 2, 2 },
        {13, 13, 13, 13, 13, 12, 12, 12 },
    },
    { // green
        {13, 12, 12, 13, 13, 13, 14, 14, },
        {12, 4, 4, 4, 13, 13, 2, 2, },
        {13, 4, 4, 4, 13, 13, 12, 12, },
        {13, 4, 4, 4, 13, 13, 13, 13, },
        {12, 13, 13, 13, 13, 12, 5, 5, },
        {13, 13, 13, 13, 13, 12, 5, 5, },
        {12, 13, 12, 13, 13, 12, 3, 3, },
        {13, 13, 13, 13, 13, 12, 12, 12, },
    },
    { // blue
        {12, 12, 12, 14, 13, 13, 14, 14, },
        {12, 2, 2, 2, 13, 13, 2, 2, },
        {13, 2, 2, 1, 13, 13, 12, 12, },
        {12, 2, 2, 2, 13, 13, 13, 13, },
        {13, 13, 12, 12, 13, 12, 9, 8, },
        {12, 13, 12, 12, 12, 12, 8, 8, },
        {12, 12, 12, 12, 13, 12, 4, 4, },
        {12, 13, 12, 13, 13, 12, 12, 12, },
    }
};
*/


#endif /* PATTERNS_H_ */
