//#define F_CPU 16000000L

#include <avr/io.h>
#include <avr/interrupt.h>

#define PIN_DATA   PB0 // DS
#define PIN_CLOCK  PB1 // SH_CP
#define PIN_LATCH  PB2 // ST_CP

// From Android.h
#define LSBFIRST 0
#define MSBFIRST 1

/********************************************************************************
Function Prototypes
********************************************************************************/
void sandpit();
void countTime();
void shiftOut(uint8_t bitOrder, uint8_t val);

void shiftRow(char out);
void draw();
unsigned long millis();

/********************************************************************************
Global Variables
********************************************************************************/
char cylonDirection; // the current direction the cylon sweep is moving.
char shift; // the data to be shifted
char anodes = 0b00000001;
char cathodes_a; //the first byte of cathodes
char pattern_index; // which pattern is current
unsigned long lastSecond;
unsigned long prev_millis;

static const char patterns[] = {
	0b01111111,
	0b10111111,
    0b11011111,
    0b10111111,
    0b01111111,
    0b10111111,
    0b11011111,
    0b10111111,
};

volatile char timo_ovf_register; // snapshot of the timer cycle position 0b0100 = 1ms
volatile unsigned long timo_millis;

/********************************************************************************
Interupt Routines
********************************************************************************/

// timer overflow occur every 0.256 ms
ISR(TIM0_OVF_vect) {
	// when timo_ovf_register reaches 4 then 1ms has passed (near enough)
	timo_ovf_register++;
	if (timo_ovf_register == 4) {
		timo_millis++;
		// reset to 0
		timo_ovf_register = 0;
	}
}


/********************************************************************************
Main
********************************************************************************/
int
main (void)
{
    DDRB = 0xFF; // set all to output
	PORTB = 0; // all off

	//PORTB |= _BV(PB4); // turn the counter debug LED on

	// Cathode 0 = ON, 1 = OFF
	// RGB RGB RG
	//cathodes_a = 0b01101111;
	//cathodes_a = 0; // all ON
	cathodes_a = 0b11011011;
	//cathodes_a = 0b00011111;



	// 8 bit timer in mormal mode (attiny85 datasheet 11.7.1)
	// 1 mhz F_CPU
	// prescaler 1 (its a slow clock at 1mhz)
	// 1 / (1000000 / 1 / 256) = 0.000256 = 0.256ms
	// CS00 sets the prescaler to 1 (attiny85 datasheet 11.9.3)


	// if running at 8mhz...
	// 1 / (8000000 / 8 / 256) = 0.000256 = 0.256ms
	// CS01 sets the prescaler to 8 (attiny85 datasheet 11.9.3)
	// prescale timer0 to 1/8th the clock rate
	// overflow timer0 every 0.256 ms

	TCCR0B |= (1<<CS00);
	// enable timer overflow interrupt
	TIMSK  |= 1<<TOIE0;

	// Enable global interrupts
	sei();


    while(1) {

		// every 1000ms toggle LED
    	unsigned long now = millis();
    	if (now - lastSecond > 1000) {
    		lastSecond = now;
			// Toggle Port B pin 4 output state
			PORTB ^= 1<<PB4;
		}


    	if (now - prev_millis > 1000) {

    		pattern_index++;
    		if (pattern_index > 7) {
    		    pattern_index = 0;
    		}
    	}

        //countTime();
    	//cylon();
    	//sandpit();
    }
}


/********************************************************************************
Functions
********************************************************************************/

void sandpit()
{


	//draw();

	//_delay_ms(1);
}

void draw() {

	uint8_t i;
	for (i = 0; i < 8; i++)  {
		shiftRow(patterns[i]);
		//_delay_ms(250);
	}



	/*
	shiftRow(0b01111111); _delay_ms(250);
	shiftRow(0b01111111); _delay_ms(250);
	shiftRow(0b01111111); _delay_ms(250);
	shiftRow(0b01111111); _delay_ms(250);

	shiftRow(0b10111111); _delay_ms(250);
	shiftRow(0b10111111); _delay_ms(250);
    shiftRow(0b10111111); _delay_ms(250);
    shiftRow(0b10111111); _delay_ms(250);
    */
}

void shiftRow(char out) {


	if (anodes == 0) {
	    anodes = 1;
	} else {

	}

	// Latch LOW
	PORTB &= ~(1 << PIN_LATCH);

	shiftOut(LSBFIRST, out);
    shiftOut(LSBFIRST, anodes);

	//take the latch pin high
	PORTB |= (1 << PIN_LATCH);

	anodes <<= 1;
}

void shiftOut(uint8_t bitOrder, uint8_t val)
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

			//digitalWrite(clockPin, HIGH);
        	PORTB |= (1 << PIN_CLOCK); // HIGH

			//digitalWrite(clockPin, LOW);
        	PORTB &= ~(1 << PIN_CLOCK); // LOW
        }
}

unsigned long millis()
{
	/*
    unsigned long millis_return;
    // Ensure this cannot be disrupted
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        millis_return = timer1_millis;
    }
    */
    return timo_millis;
}

void countTime()
{
	for (int8_t x = 0; x <= 255; x++) {
		// take the latchPin low so
		// the LEDs don't change while you're sending in bits:
		//digitalWrite(latchPin, LOW);
		PORTB &= ~(1 << PIN_LATCH); // LOW

		// shift out the bits:
		shiftOut(LSBFIRST, x);

		//take the latch pin high so the LEDs will light up:
        //digitalWrite(latchPin, HIGH);
		PORTB |= (1 << PIN_LATCH); // HIGH


	}
}


