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

#define COMPARE_REG 50 // OCR0A (1/5 of 1ms = 0.0002ms) - when to interupt (datasheet: 14.9.4)
#define MILLIS_TICKS 5  // number of ISR calls before a millisecond is counted

#define T1 2 * MILLIS_TICKS // timeout value (mSec)
#define TICK_COUNTS 15 // Max number of ISR calls to count

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
uint8_t anodes = 0b11111110; // low = ON with pnp
uint8_t current_row; // Which row of the frame is currently being shown via the multiplexing.
uint8_t current_frame[8]; // The current frame being displayed
unsigned long current_frame_duration = 2000 * MILLIS_TICKS; // millis to show the current frame.
unsigned long current_frame_start; // When the current frame was first shown.

uint8_t current_letter; // tmp: the index of font to show

uint8_t font[2][8] = {
  {0xfe,0xfe,0x10,0x10,0xfe,0xfe,0x00,0x00}, // H
  {0x00,0x82,0xfe,0xfe,0x82,0x00,0x00,0x00}, // I
};

volatile uint8_t tick_count;
volatile unsigned int time1;
volatile unsigned int frame_time;

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
    if (tick_count > 0)  --tick_count;

    // Shift out prepared data (red, blue, green, anodes) gets shifted out at this regular interval.
    shiftData();
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

    	if (tick_count == 0) {
    		tick_count = TICK_COUNTS;
    	}

    	if (time1 == 0) {
    		// reset the timer
    		time1 = T1;

    		//shiftData();
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
	//OCR0A = 50; // 1/5 of 1ms = 0.0002ms
	OCR0A = COMPARE_REG;

    // Timer mode (datasheet: 14.9.1)
	TCCR0A = (1 << WGM01); // (0b00000010) turn on clear-on-match

	// Prescaler (datasheet: 14.9.2)
	// calculation to show why 64 is required the prescaler:
	// 1 / (16000000 / 64 / 250) = 0.001 = 1ms
	TCCR0B = ((1 << CS10) | (1 << CS11)); // (0b00000011)(3) clock prescalar to 64

	// Timer initialization
    time1 = T1;
    frame_time = current_frame_duration;
    tick_count = TICK_COUNTS;

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
	// Turn off the leds with OE high
	// To prevent ghosting, while the data gets shifted.
    PORTC |= (1 << PIN_OE);

    // Latch LOW
    // take the latchPin low so the LEDs don't change sending in bits:
	PORTB &= ~(1 << PIN_LATCH);

	// NOT the bytes so the patterns are 1 == ON 0 == OFF
	//if (tick_count > 12) {
	    shiftByte(MSBFIRST, ~current_frame[current_row]);
	//} else {
	//	shiftByte(MSBFIRST, ~0);
	//}
	shiftByte(MSBFIRST, ~current_frame[current_row]);
	// YEP, the hardware I built needs the green to be least significate bit first :(
	//if (tick_count > 8) {
	   shiftByte(LSBFIRST, ~current_frame[current_row]);
    //} else {
	//	shiftByte(MSBFIRST, ~0);
	//}

	shiftByte(MSBFIRST, anodes);


	// Take the latch pin high, to move the shifted data into position
	PORTB |= (1 << PIN_LATCH);

	// OE low to turn on the leds
	PORTC &= ~(1 << PIN_OE);

	// Get the anodes ready for the next call
	// At the end of the register add on bits to the start again.
	if (anodes & (1 << 7)) {
		// Shift left
		anodes = (anodes << 1);
		// Add one to the start (wrap around)
		anodes |= (1 << 0);
	} else {
		// Shift left
		anodes = (anodes << 1);
	}

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

