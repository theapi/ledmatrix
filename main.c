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

// 32 * 0.000004 * 8 = 0.001024 = about 1khz (NB zero based so register one less)
#define COMPARE_REG 31 // OCR0A when to interupt (datasheet: 14.9.4)
#define MILLIS_TICKS 8  // number of ISR calls before a millisecond is counted (ish)

#define T1 1 * MILLIS_TICKS // timeout value (mSec)
#define COUNT_CYCLE_TICK 8 // number of ISR calls in a cycle
#define COUNT_CYCLE 16 // number of cycles needed for 4bit bit angle modulation

/********************************************************************************
Function Prototypes
********************************************************************************/

void initialize(void); //all the usual mcu stuff
void setFrame(uint8_t pattern[8]);
void shiftData();
void shiftByte(uint8_t bitOrder, uint8_t val);



/********************************************************************************
Global Variables
********************************************************************************/

volatile uint8_t cycle_count; // keeps track of the number of times a complete multiplex loop has happened.
//volatile uint8_t anodes = 0b11111110; // low = ON with pnp
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

volatile uint8_t current_row; // Which row of the frame is currently being shown via the multiplexing.
volatile uint8_t current_frame[8]; // The current frame being displayed
unsigned long current_frame_duration = 2000 * MILLIS_TICKS; // millis to show the current frame.
unsigned long current_frame_start; // When the current frame was first shown.

uint8_t current_letter; // tmp: the index of font to show

uint8_t font[2][8] = {
  {0xfe,0xfe,0x10,0x10,0xfe,0xfe,0x00,0x00}, // H
  {0x00,0x82,0xfe,0xfe,0x82,0x00,0x00,0x00}, // I
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
	// 75 cycles in
	// 32 cycles save cpu state
	// 8 cycles overhead
	// 32 cycles resume cpu state
	// 75 cycles out

	// Decrement the time if not already zero
    if (time1 > 0)  --time1;
    if (frame_time > 0)  --frame_time;
    if (cycle_tick_count > 0)  --cycle_tick_count;

    // Shift out prepared data (red, blue, green, anodes) gets shifted out at this regular interval.
    shiftData();

/*
    // Data is (should be) ready to be latched
    // Turn off the leds with OE high
    PORTC |= (1 << PIN_OE);

    // Latch low to provide an edge
    PORTB &= ~(1 << PIN_LATCH);
    // Latch high to move the shifted data into position
    PORTB |= (1 << PIN_LATCH);

    // Output Enable low to turn on the leds
    PORTC &= ~(1 << PIN_OE);
*/

    // allow new data to be shifted
    data_shifted = 0;
}


/********************************************************************************
Main
********************************************************************************/
int
main (void)
{

	DDRC = 0xFF; // set all to output
	PORTC |= (1 << PIN_OE); // Output enable high so the display is off

	DDRB = 0xFF; // set all to output
	PORTB = 0; // all off

	setFrame(font[current_letter]);

	initialize();

	PORTC &= ~(1 << PIN_OE); // leds on

	// main loop
    while(1) {


    	// check for countdown reached 0
    	if (frame_time == 0) {
    		// reset the frame timer
    		frame_time = current_frame_duration;

			if (current_letter == 1) {
			  current_letter = 0;
			} else {
			  current_letter = 1;
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

    		//shiftData();
    	}

    	if (!data_shifted) {
    		// Shift the data ready to be latched in the ISR
    		//shiftData();
    		data_shifted = 1;
    	}

    }
}


/********************************************************************************
Functions
********************************************************************************/

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

void setFrame(uint8_t pattern[8])
{
  uint8_t i;
  for (i = 0; i < 8; i++) {
    current_frame[i] = pattern[i];
  }
}

/**
 * Shift out the prepared data.
 */
void shiftData()
{

	// Latch LOW
	PORTB &= ~(1 << PIN_LATCH);

	// Turn off the leds with OE high
	// To prevent ghosting/leakage.
	// The process seems to need time to settle (parasitic capcitance?)
	// The data shifting takes long enough for the settling to occur.
	// This is why the the data is shifted in the ISR as well as the latching.
    PORTC |= (1 << PIN_OE);


	// NOT (invert) the bytes so the patterns are 1 == ON 0 == OFF

	// pretend bit angle modulation - on for half the full number of 16 cycles
	if (cycle_count > 4) {
	    shiftByte(MSBFIRST, ~current_frame[current_row]);
	} else {
		shiftByte(MSBFIRST, ~0);
	}
	shiftByte(MSBFIRST, ~current_frame[current_row]);
	// YEP, the hardware I built needs the green to be least significate bit first :(
	if (cycle_count > 8) {
	   shiftByte(LSBFIRST, ~current_frame[current_row]);
    } else {
		shiftByte(LSBFIRST, ~0);
	}

	shiftByte(MSBFIRST, anodes[current_row]);

	++current_row;
	if (current_row > 7) {
		current_row = 0;
	}

    // Take the latch pin high, to move the shifted data into position
	PORTB |= (1 << PIN_LATCH);

	// OE low to turn on the leds
	PORTC &= ~(1 << PIN_OE);
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

