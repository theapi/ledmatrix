//#define F_CPU 16000000L

#include <avr/io.h>
#include <avr/interrupt.h>


#define PIN_DATA   PB3 // DS - 11
#define PIN_CLOCK  PB5 // SH_CP - SCK - 13
#define PIN_LATCH  PB4 // ST_CP - 12
#define PIN_OE     PC3 // Output Enable - 17

// From Android.h
#define LSBFIRST 0
#define MSBFIRST 1

// 32 * 0.000004 = 0.000128 so ISR gets called every 128us
// 32 * 0.000004 * 8 = 0.001024 = about 1khz for whole matrix (NB zero based so register one less)
#define COMPARE_REG 31 // OCR0A when to interupt (datasheet: 14.9.4)
#define MILLIS_TICKS 8  // number of ISR calls before a millisecond is counted (ish)

#define T1 1 * MILLIS_TICKS // timeout value (mSec)
#define COUNT_CYCLE_TICK 8 // number of ISR calls in a cycle
#define COUNT_CYCLE 16 // number of cycles needed brightness

/********************************************************************************
Function Prototypes
********************************************************************************/

void initialize(void); //all the usual mcu stuff
void setFrame(uint8_t pattern[8]);
void latchLow();
void latchHigh();
void ledsDisable();
void ledsEnable();
void shiftData();
void shiftByte(uint8_t bitOrder, uint8_t val);



/********************************************************************************
Global Variables
********************************************************************************/

uint8_t cycle_count; // keeps track of the number of times a complete multiplex loop has happened.
uint8_t current_row; // Which row of the frame is currently being shown via the multiplexing.
uint8_t current_frame[8]; // The current frame being displayed
unsigned long current_frame_duration = 2000 * MILLIS_TICKS; // millis to show the current frame.
unsigned long current_frame_start; // When the current frame was first shown.

uint8_t current_letter; // tmp: the index of font to show

// anode low = ON with pnp
uint8_t anodes[8] = {
  0b11111110,
  0b11111101,
  0b11111011,
  0b11110111,
  0b11101111,
  0b11011111,
  0b10111111,
  0b01111111,
};

// https://github.com/dhepper/font8x8/blob/master/font8x8_basic.h
uint8_t font[26][8] = {
  //{0xfe,0xfe,0x10,0x10,0xfe,0xfe,0x00,0x00}, // H
  //{0x00,0x82,0xfe,0xfe,0x82,0x00,0x00,0x00}, // I

{ 0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00},   // U+0041 (A)
    { 0x3F, 0x66, 0x66, 0x3E, 0x66, 0x66, 0x3F, 0x00},   // U+0042 (B)
    { 0x3C, 0x66, 0x03, 0x03, 0x03, 0x66, 0x3C, 0x00},   // U+0043 (C)
    { 0x1F, 0x36, 0x66, 0x66, 0x66, 0x36, 0x1F, 0x00},   // U+0044 (D)
    { 0x7F, 0x46, 0x16, 0x1E, 0x16, 0x46, 0x7F, 0x00},   // U+0045 (E)
    { 0x7F, 0x46, 0x16, 0x1E, 0x16, 0x06, 0x0F, 0x00},   // U+0046 (F)
    { 0x3C, 0x66, 0x03, 0x03, 0x73, 0x66, 0x7C, 0x00},   // U+0047 (G)
    { 0x33, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33, 0x00},   // U+0048 (H)
    { 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+0049 (I)
    { 0x78, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E, 0x00},   // U+004A (J)
    { 0x67, 0x66, 0x36, 0x1E, 0x36, 0x66, 0x67, 0x00},   // U+004B (K)
    { 0x0F, 0x06, 0x06, 0x06, 0x46, 0x66, 0x7F, 0x00},   // U+004C (L)
    { 0x63, 0x77, 0x7F, 0x7F, 0x6B, 0x63, 0x63, 0x00},   // U+004D (M)
    { 0x63, 0x67, 0x6F, 0x7B, 0x73, 0x63, 0x63, 0x00},   // U+004E (N)
    { 0x1C, 0x36, 0x63, 0x63, 0x63, 0x36, 0x1C, 0x00},   // U+004F (O)
    { 0x3F, 0x66, 0x66, 0x3E, 0x06, 0x06, 0x0F, 0x00},   // U+0050 (P)
    { 0x1E, 0x33, 0x33, 0x33, 0x3B, 0x1E, 0x38, 0x00},   // U+0051 (Q)
    { 0x3F, 0x66, 0x66, 0x3E, 0x36, 0x66, 0x67, 0x00},   // U+0052 (R)
    { 0x1E, 0x33, 0x07, 0x0E, 0x38, 0x33, 0x1E, 0x00},   // U+0053 (S)
    { 0x3F, 0x2D, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+0054 (T)
    { 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3F, 0x00},   // U+0055 (U)
    { 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00},   // U+0056 (V)
    { 0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00},   // U+0057 (W)
    { 0x63, 0x63, 0x36, 0x1C, 0x1C, 0x36, 0x63, 0x00},   // U+0058 (X)
    { 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x0C, 0x1E, 0x00},   // U+0059 (Y)
    { 0x7F, 0x63, 0x31, 0x18, 0x4C, 0x66, 0x7F, 0x00},   // U+005A (Z)


};

volatile uint8_t cycle_tick_count;
volatile unsigned int time1;
volatile unsigned int frame_time;
volatile uint8_t data_shifted; // whether the data has been shifted and just needs to be latched.

/********************************************************************************
Interupt Routines
********************************************************************************/

//timer 0 compare ISR
ISR (TIMER0_COMPA_vect)
{
	// Do as little as possible in this ISR.

	// Just latch the data.
	ledsDisable();
    latchLow();
    latchHigh();
    // Flag that the data has been latched & enabled.
    data_shifted = 0;

	// Decrement the time if not already zero
    if (time1 > 0)  --time1;
    if (frame_time > 0)  --frame_time;
    if (cycle_tick_count > 0)  --cycle_tick_count;

/*
    // Dumbass delay because time is needed to reduce ghosting.
    uint8_t j = 5;
    while(--j) {
    	ledsDisable();
    }
*/

    // Now enable the leds again.
    ledsEnable();
}


/********************************************************************************
Main
********************************************************************************/
int
main (void)
{

	DDRC = 0xFF; // set all to output
	ledsDisable(); // Output enable high so the display is off

	DDRB = 0xFF; // set all to output
	PORTB = 0; // all off

	setFrame(font[current_letter]);

	initialize();

	ledsEnable(); // leds on

	data_shifted = 0;

	// main loop
    while(1) {

    	// check for countdown reached 0
    	if (frame_time == 0) {
    		// reset the frame timer
    		frame_time = current_frame_duration;
    		current_letter++;

			if (current_letter > 25) {
			  current_letter = 0;
			}
			setFrame(font[current_letter]);
    	}

    	if (cycle_tick_count == 0) {
    		cycle_tick_count = COUNT_CYCLE_TICK;
    		// decrement the cycle count as one cycle has completed
    		if (cycle_count > 0) {
    			--cycle_count;
    		} else {
    			// reset the cycle counter
    			cycle_count = COUNT_CYCLE;
    		}
    	}

    	if (time1 == 0) {
    		// reset the timer
    		time1 = T1;
    	}

    	if (!data_shifted) {
    		// Shift the data ready to be latched in the ISR
			shiftData();
			// Flag that the data is ready to be latched in the ISR.
    		data_shifted = 1;
    	}

    }
}


/********************************************************************************
Functions
********************************************************************************/

/**
 * Make least significant bit highest etc.
 */
uint8_t flipByte(uint8_t byte)
{
	return byte;
	uint8_t i;
	uint8_t flipped = byte;

	for (i = 0; i < 8; i++)  {
		if (byte & (1 << i)) {
			flipped |= (1 << i); // 0
		} else {
			flipped &= ~(1 << i); // 1
		}
	}

	return flipped;
}

/**
 * Set the latch low.
 */
void latchLow()
{
	PORTB &= ~(1 << PIN_LATCH);
}

/**
 * Take the latch pin high, to move the shifted data into position
 */
void latchHigh()
{
	PORTB |= (1 << PIN_LATCH);
}

/**
 * OE low to turn on the leds.
 */
void ledsEnable()
{
	PORTC &= ~(1 << PIN_OE);
}

/**
 * Turn off the leds with OE high.
 *
 * To prevent ghosting/leakage.
 * The process seems to need time to settle (parasitic capcitance?)
 */
void ledsDisable()
{
	PORTC |= (1 << PIN_OE);
}

/**
 * Shift out the prepared data.
 */
void shiftData()
{
	// NOT (invert) the bytes so the patterns are 1 == ON 0 == OFF

	// RED
	if (cycle_count > 14) {
	shiftByte(MSBFIRST, ~flipByte(current_frame[current_row]));
	} else {
		shiftByte(MSBFIRST, ~0);
	}
	// BLUE
	if (cycle_count > 8) {
	shiftByte(MSBFIRST, ~flipByte(current_frame[current_row]));
	} else {
		shiftByte(MSBFIRST, ~0);
	}
	// GREEN
	// YEP, the hardware I built needs the green to be least significate bit first :(
	shiftByte(LSBFIRST, ~flipByte(current_frame[current_row]));

	// ANODES
	shiftByte(MSBFIRST, anodes[current_row]);

	++current_row;
	if (current_row > 7) {
		current_row = 0;
	}
}

void shiftByte(uint8_t bitOrder, uint8_t val)
{
	uint8_t i;
	uint8_t bit;

	for (i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST) {
			//digitalWrite(dataPin, !!(val & (1 << i)));
			bit = i;
		} else {
			//digitalWrite(dataPin, !!(val & (1 << (7 - i))));
			bit = 7 - i;
		}

		if (val & (1 << bit)) {
			PORTB |= (1 << PIN_DATA); // HIGH
		} else {
			PORTB &= ~(1 << PIN_DATA); // LOW
		}

		// Clock in the bit
		PORTB |= (1 << PIN_CLOCK); // HIGH
		PORTB &= ~(1 << PIN_CLOCK); // LOW
	}
}

void setFrame(uint8_t pattern[8])
{
  uint8_t i;
  for (i = 0; i < 8; i++) {
    current_frame[i] = pattern[i];
  }
}

void initialize(void)
{
	// set up timer 0 for 1 mSec ticks (timer 0 is an 8 bit timer)

	// Interupt mask register - to enable the interupt (datasheet: 14.9.6)
	// (Bit 1 – OCIE0A: Timer/Counter0 Output Compare Match A Interrupt Enable)
	TIMSK0 = (1 << OCIE0A); // (2) turn on timer 0 cmp match ISR

	// Compare register - when to interupt (datasheet: 14.9.4)
	// OCR0A = 249; // set the compare reg to 250 time ticks = 1ms
	//OCR0A = 124; // 0.5 ms
	//OCR0A = 50; // 1/5 of 1ms = 0.0002s
	//OCR0A = 30; // 0.00012s = 0.12ms
	//OCR0A = 25; // 0.0001s = 0.1ms
	OCR0A = COMPARE_REG;

    // Timer mode (datasheet: 14.9.1)
	TCCR0A = (1 << WGM01); // (0b00000010) turn on clear-on-match

	// Prescaler (datasheet: 14.9.2)
	// 16MHz/64=250kHz so precision of 0.000004 = 4us
	// calculation to show why 64 is required the prescaler:
	// 1 / (16000000 / 64 / 250) = 0.001 = 1ms
	TCCR0B = ((1 << CS10) | (1 << CS11)); // (0b00000011)(3) clock prescalar to 64

	// Timer initialization
    time1 = T1;
    frame_time = current_frame_duration;
    cycle_tick_count = COUNT_CYCLE_TICK;
    cycle_count = COUNT_CYCLE;

	// crank up the ISRs
	sei();
}
