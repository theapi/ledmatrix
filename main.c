//#define F_CPU 16000000L


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>



#include "USART.h"
#include "matrix.h"
#include "frame.h"



// 32 * 0.000004 = 0.000128 so ISR gets called every 128us
// 32 * 0.000004 * 8 = 0.001024 = about 1khz for whole matrix (NB zero based so register one less)
#define COMPARE_REG 31 // OCR0A when to interupt (datasheet: 14.9.4)
#define MILLIS_TICKS 8  // number of ISR calls before a millisecond is counted (ish)

#define T1 1 * MILLIS_TICKS // timeout value (mSec)



#define SOURCE_SIZE_PATTERN 7 // The number of items in the pattern source array.

#define SOURCE_SIZE_FONT    234 // The number of items in the font source array.
#include "font.h"

/********************************************************************************
Function Prototypes
********************************************************************************/

void timerInit(void); //all the usual mcu stuff
void initSPI(void);

void rxProcess(void);
void buildImageFromString(uint8_t image[][8][8], uint8_t str[]);


/********************************************************************************
Global Variables
********************************************************************************/

uint8_t cycle_count; // keeps track of the number of times a complete multiplex loop has happened.
uint8_t current_row; // Which row of the frame is currently being shown via the multiplexing.
uint8_t current_frame[3][8]; // The current frame being displayed
uint8_t current_frame_coloured[3][8][8];
uint8_t image[3][8][8]; // A coloured image

unsigned long current_frame_duration = 2000 * MILLIS_TICKS; // millis to show the current frame.

// States for the usart receive processing.
enum rx_states {RX_COMMAND, RX_ARGS};
enum rx_states rx_state = RX_COMMAND;
uint8_t rx_cmd;  // The command being requested by serial data.
uint8_t rx_args; // The argument for rx_cmd being requested by serial data.


uint8_t source_array; // The array that is the source of data.
uint8_t source_index; // The source array index that is currently being shown.


uint8_t pattern[SOURCE_SIZE_PATTERN][8] PROGMEM = {
    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFF}, // |_
    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFF, 0xFF}, // |_
    {0x80, 0x80, 0x80, 0x80, 0x80, 0xFF, 0x80, 0xFF}, // |_
    {0x00, 0x66, 0x66, 0x00, 0x42, 0x42, 0x3C, 0x00}, // :)
    {0x00, 0x06, 0x66, 0x00, 0x42, 0x42, 0x3C, 0x00}, // ;)
    {0x00, 0x00, 0x66, 0x00, 0x42, 0x42, 0x3C, 0x00}, // |)
    {0x00, 0x66, 0x66, 0x00, 0x7E, 0x7E, 0x3C, 0x00}, // :O
};


uint8_t imgStr[] = "13,13,12,13,12,12,13,12,12,13,13,14,13,13,13,14,13,13,14,14,14,15,14,14,\
13,12,12,12,4,2,11,4,2,11,4,2,13,13,13,13,13,13,15,14,8,15,14,8,\
13,13,13,11,4,2,11,4,2,12,4,1,13,13,13,13,13,13,13,12,12,13,12,12,\
13,13,12,11,4,2,11,4,2,12,4,2,13,13,13,13,13,13,13,13,13,13,13,13,\
14,12,13,13,13,13,13,13,12,13,13,12,13,13,13,12,12,12,2,5,9,2,5,8,\
13,13,12,13,13,13,13,13,12,13,13,12,13,13,12,12,12,12,2,5,8,2,5,8,\
13,12,12,13,13,12,13,12,12,13,13,12,13,13,13,12,12,12,2,3,4,2,3,4,\
13,13,12,13,13,13,13,13,12,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,";

/*
uint8_t imgStr[] = "9,9,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,\
9,9,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,\
9,9,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,\
9,9,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,\
9,9,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,\
9,9,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,\
9,9,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,\
9,9,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,";
*/



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


volatile unsigned int time1;
volatile unsigned int frame_time;
volatile uint8_t data_sent; // whether the data has been sent and just needs to be latched.



/********************************************************************************
Interupt Routines
********************************************************************************/

//timer 0 compare ISR
ISR (TIMER0_COMPA_vect)
{
	// Do as little as possible in this ISR.
    if (data_sent) {
        // Just latch the data.
        matrix_ledsDisable();
        matrix_latchLow();
        matrix_latchHigh();

        // Reset the flag so new data can now be sent.
        data_sent = 0;


        // Decrement the time if not already zero
        if (time1 > 0)  --time1;
        if (frame_time > 0)  --frame_time;

    /*
        // Dumbass delay because time is needed to reduce ghosting.
        uint8_t j = 5;
        while(--j) {
            ledsDisable();
        }
    */

        // Now enable the leds again.
        matrix_ledsEnable();
    }
}



/********************************************************************************
Main
********************************************************************************/
int
main (void)
{

	DDRC = 0xFF; // set all to output
	matrix_ledsDisable(); // Output enable high so the display is off

	DDRB = 0xFF; // set all to output
	PORTB = 0; // all off

	USART_Init();
	initSPI();

	timerInit();

	matrix_ledsEnable(); // leds on

	data_sent = 0;

	// crank up the ISRs
	sei();

	buildImageFromString(image, imgStr);
	frame_SetColoured(current_frame_coloured, image[0], image[1], image[2]);

	// main loop
    while(1) {

        // Handle unprocessed received serial data.
        rxProcess();

    	if (time1 == 0) {
    		// reset the timer
    		time1 = T1;

    	}

    	// check for countdown reached 0
    	if (frame_time == 0) {
    		// reset the frame timer
    		frame_time = current_frame_duration;

/*
    		if (source_array == 'f') {
    		    if (source_index >= SOURCE_SIZE_FONT) {
                    source_index = 0;
                }
    		    frame_SetMono_P(current_frame, font[source_index], font[source_index], font[source_index]);
    		    // Currently the font needs to be rotated.
    		    // @todo pre-rotate the font
    		    frame_Rotate(270);
    		} else {
    		    if (source_index >= SOURCE_SIZE_PATTERN) {
    		        source_index = 0;
    		    }
    		    frame_SetMono_P(current_frame, pattern[source_index], pattern[source_index], pattern[source_index]);
    		    //source_index++;
    		}
    		source_index++;
*/
    	}

    	if (!data_sent) {

    	    // current_frame_coloured
    	    uint8_t i;
    	    uint8_t r = 0;
    	    uint8_t g = 0;
    	    uint8_t b = 0;
    	    for (i=0; i<8; i++) {
    	        if (current_frame_coloured[0][current_row][i] <= cycle_count) {
    	            r |= (1 << i);
    	        }
    	        if (current_frame_coloured[1][current_row][i] <= cycle_count) {
                    g |= (1 << i);
                }
    	        if (current_frame_coloured[2][current_row][i] <= cycle_count) {
                    b |= (1 << i);
                }
    	    }

    	    matrix_sendLine(r,g,b);
    	    /*
            // Send the next line ready to be latched in the ISR
            matrix_sendLine(
                ~current_frame[0][current_row], // Red
                ~current_frame[1][current_row], // Green
                ~current_frame[2][current_row]  // Blue
            );
             */

    	    // Prepare for the next row.
    	    ++current_row;
            if (current_row > 7) {
                current_row = 0;

                // One cycle completed.
                ++cycle_count;
                if (cycle_count >= MAX_CYCLE_COUNT) {
                    cycle_count = 0;
                }

            }

			// Flag that the data is ready to be latched in the ISR.
    		data_sent = 1;
    	}

    }
}


/********************************************************************************
Functions
********************************************************************************/

void buildImageFromString(uint8_t image[][8][8], uint8_t str[])
{
    int i = 0; // index of the string array
    uint8_t state = 0;
    uint8_t col = 0;
    uint8_t row = 0;
    uint8_t num = 0;
    uint8_t c = 0;

    while(str[i] != 0) {
        c = str[i];
        //USART_Transmit(c);

        // Only interested in numbers as they make up the index of the array to use.
        // ASCII hex values 0x30 to 0x30 are decimal 0 to 9
        if (c > 0x2F && c < 0x3A) {
            if (num > 0) {
                // Expecting decimal numbers like 24,
                // so for each new argument multiply by ten (shift left in decimal).
                num *= 10;
                num += c - 0x30;
            } else {
                num = c - 0x30;
            }

        } else if (c == ',') { // Comma denotes next colour.
            USART_Transmit(num);
            if (state == 0) { // red
                image[0][row][col] = num;
            } else if (state == 1) { // green (1 is a comma)
                image[1][row][col] = num;
            } else if (state == 2) { // blue (3 is a comma)
                image[2][row][col] = num;
            }
            // Next colour
            num = 0;
            state++;
        }

        // r,g,b,
        if (state > 2) {
            state = 0;
            // Start building next pixel.
            col++;
        }

        if (col > 7) {
            col = 0;
            row++;
        }

        if (row > 7) {
            return;
        }

        i++;
    }
}

void timerInit(void)
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
}

void initSPI(void)
{
	SPCR = (1<<SPE) | (1<<MSTR);	//Start SPI as Master
	SPSR = (1<<SPI2X); // double speed


	/*
	//set up SPI control register SPCR
    //bit 7 SPIE=0 no ISR
    //bit 6 SPE=1 enable spi
    //bit 5 DORD=0 msb first
    //bit 4 MSTR=1 spi master
    //bit 3 CPOL=1 clock polarity
    //bit 2 CPHA=1 clock phase
    //bit 1,0 rate sel=00 along with SPSR.0=1 sets clk to f/2 = 8 MHz

	//SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPOL) | (1<<CPHA) ;
	SPCR = 0b01110000 ;

    //SPSR = (1<<SPI2X) ;
	SPSR = 1;
*/
}

/**
 * State machine to handle the incoming serial data.
 *
 * The command is only one byte.
 * (uint8_t rx_cmd)args\n
 * eg:
 * f23\n
 */
void rxProcess(void)
{
    if (USART_Empty()) {
        // Nothing to do.
        return;
    }

    uint8_t c = USART_ReadByte();

    USART_Transmit(c);

    if (rx_state == RX_COMMAND) {
        // reset the command arguments
        rx_args = 0;
        // Commands are only one byte.
        rx_cmd = c;
        // Now get the arguments.
        rx_state = RX_ARGS;
    } else {
        // rx_state == RX_ARGS

        // Arguments are terminated by new line.
        if (c == '\n') {
            // Newline so execute the command
            // Only one command for now: 'f' is the command for font
            if (rx_cmd == 'f' || rx_cmd == 'p') {
                source_array = rx_cmd;
                source_index = rx_args;
                // Make it display now.
                frame_time = 0;
            }

            rx_state = RX_COMMAND;
        } else {

            if (rx_cmd == 'f' || rx_cmd == 'p') {
                // Only interested in numbers as they make up the index of the array to use.
                // ASCII hex values 0x30 to 0x30 are decimal 0 to 9
                if (c > 0x2F && c < 0x3A) {
                    if (rx_args > 0) {
                        // Expecting decimal numbers like 24,
                        // so for each new argument multiply by ten (shift left in decimal).
                        rx_args *= 10;
                        rx_args += c - 0x30;
                    } else {
                        rx_args = c - 0x30;
                    }
                }
            }
        }
    }

}

