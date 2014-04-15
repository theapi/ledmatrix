//#define F_CPU 16000000L


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "USART.h"
#include "matrix.h"
#include "frame.h"
#include "patterns.h"
#include "font.h"
#include "scroll.h"
#include "anim.h"

// 32 * 0.000004 = 0.000128 so ISR gets called every 128us
// 32 * 0.000004 * 8 = 0.001024 = about 1khz for whole matrix (NB zero based so register one less)
#define COMPARE_REG 31 // OCR0A when to interupt (datasheet: 14.9.4)
#define MILLIS_TICKS 8  // number of ISR calls before a millisecond is counted (ish)
#define T1 125 * MILLIS_TICKS // timeout value (mSec)



/********************************************************************************
Function Prototypes
********************************************************************************/

// Configure the timer interupt.
void timerInit(void);
// Configure the SPI.
void initSPI(void);

void rxProcess(void);
//void buildImageFromString(uint8_t image[][8][8], uint8_t str[]);
uint8_t rxBuildImage(uint8_t image[][8][8], uint8_t c);

/********************************************************************************
Global Variables
********************************************************************************/

uint8_t cycle_count; // keeps track of the number of times a complete multiplex loop has happened.
uint8_t current_row; // Which row of the frame is currently being shown via the multiplexing.
uint8_t current_frame[3][8]; // The current frame being displayed
uint8_t source_buffer[3][8][8]; // The frame that is being scrolled/merged into the current one
uint8_t scrolled = 0; // How many pixels scrolled
uint8_t current_frame_coloured[3][8][8];
uint8_t image[3][8][8]; // A coloured image

unsigned long current_frame_duration = 2000 * MILLIS_TICKS; // millis to show the current frame.

// States for the usart receive processing.
enum rx_states {RX_COMMAND, RX_ARGS};
enum rx_states rx_state = RX_COMMAND;
uint8_t rx_cmd;  // The command being requested by serial data.
uint8_t rx_args; // The argument for rx_cmd being requested by serial data.
uint8_t rx_verbose = 1; // Whether to send responses back when a command is sent.

uint8_t source_array; // The array that is the source of data.
uint8_t source_index; // The source array index that is currently being shown.



volatile unsigned int time1;
volatile unsigned int frame_time;
volatile uint8_t data_sent; // whether the data has been sent and just needs to be latched.

// Temporary colour for mono frames, which act as a mask.
uint8_t example_colour[3][8][8] =
{
    { // red
        {16, 2, 2, 2, 2, 2, 2, 2 },
        {2, 2, 2, 2, 2, 2, 2, 2 },
        {2, 2, 2, 2, 2, 2, 2, 2 },
        {2, 2, 2, 2, 2, 2, 2, 2 },
        {2, 2, 2, 2, 2, 2, 2, 2 },
        {2, 2, 2, 2, 2, 2, 2, 2 },
        {2, 2, 2, 2, 2, 2, 2, 2 },
        {2, 2, 2, 2, 2, 2, 2, 2 },
    },
    { // green
        {0, 5, 5, 5, 5, 5, 5, 5 },
        {5, 5, 5, 5, 5, 5, 5, 5 },
        {5, 5, 5, 5, 5, 5, 5, 5 },
        {5, 5, 5, 5, 5, 5, 5, 5 },
        {5, 5, 5, 5, 5, 5, 5, 5 },
        {5, 5, 5, 5, 5, 5, 5, 5 },
        {5, 5, 5, 5, 5, 5, 5, 5 },
        {5, 5, 5, 5, 5, 5, 5, 5 },
    },
    { // blue
        {0, 15, 15, 15, 15, 15, 15, 15 },
        {15, 15, 15, 15, 15, 15, 15, 15 },
        {15, 15, 15, 15, 15, 15, 15, 15 },
        {15, 15, 15, 15, 15, 15, 15, 15 },
        {15, 15, 15, 15, 15, 15, 15, 15 },
        {15, 15, 15, 15, 15, 15, 15, 15 },
        {15, 15, 15, 15, 15, 15, 15, 15 },
        {15, 15, 15, 15, 15, 15, 15, 15 },
    }
};

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

	frame_Colourise_P(current_frame_coloured, patterns[0], example_colour);

/*
	anim_Add(1, 250 * MILLIS_TICKS);
	//anim_SetBeggining(anim_CurrentIndex());
	anim_Add(2, 250 * MILLIS_TICKS);
	anim_Add(1, 200 * MILLIS_TICKS);
    anim_Add(3, 900 * MILLIS_TICKS);
    anim_Add(1, 250 * MILLIS_TICKS);
    anim_Add(2, 150 * MILLIS_TICKS);
    anim_Add(4, 900 * MILLIS_TICKS);
    anim_Add(3, 1000 * MILLIS_TICKS);
    //anim_SetLength(8);
*/
	anim_Add(5, 250 * MILLIS_TICKS);
	anim_Add(6, 250 * MILLIS_TICKS);
	anim_Add(5, 250 * MILLIS_TICKS);
	anim_Add(6, 250 * MILLIS_TICKS);
	anim_Add(7, 250 * MILLIS_TICKS);
	anim_Add(8, 250 * MILLIS_TICKS);
	anim_Add(9, 250 * MILLIS_TICKS);
	anim_Add(10, 250 * MILLIS_TICKS);
	anim_Add(9, 250 * MILLIS_TICKS);

	// crank up the ISRs
    sei();

	// main loop
    while(1) {

        // Handle unprocessed received serial data.
        rxProcess();

    	if (time1 == 0) {
    		// reset the timer
    		time1 = T1;

    		if (source_array == 'F') {
    		    // Scroll
    		    // @todo kerning.
    		    if (scrolled == 0) {
    		        if (scroll_Empty()) {
                        source_index = 0; // A blank screen.
                    } else {
                        source_index = scroll_Shift();
                    }
    		        //frame_SetMono_P(source_buffer, font[source_index], font[source_index], font[source_index]);
    		        frame_Colourise_P(source_buffer, font[source_index], example_colour);
    		    }

                //scroll_LeftMono(current_frame, source_buffer);
    		    scroll_Left(current_frame_coloured, source_buffer);
                scrolled++;
                if (scrolled > 7) {
                    scrolled = 0;
                }

    		}

    	}




    	// check for countdown reached 0
    	if (frame_time == 0) {

    		if (source_array == 'f') {
    		    if (source_index >= SOURCE_SIZE_FONT) {
                    source_index = 0;
                }
    		    frame_Colourise_P(current_frame_coloured, font[source_index], example_colour);
    		} else if (source_array == 'p') {
    		    if (source_index >= SOURCE_SIZE_PATTERNS) {
    		        source_index = 0;
    		    }
    		    frame_Colourise_P(current_frame_coloured, patterns[source_index], example_colour);
    		    //source_index++;
    		} else {
    		    // Play the animation.
    		    anim_Next();
    		    source_index = anim_CurrentIndex();
    		    current_frame_duration = anim_CurrentDuration();
    		    frame_Colourise_P(current_frame_coloured, patterns[source_index], example_colour);
    		}

    		// reset the frame timer
            frame_time = current_frame_duration;
    	}

    	if (!data_sent) {

            // Calculate the bits to send.
    	    // If the colour brightness number is higher than the cycle count,
    	    // the bit will be on.
    	    // So software PWM.
            uint8_t col;
            uint8_t bit = 7;
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;
            for (col=0; col<8; col++) {
                if (current_frame_coloured[0][current_row][col] <= cycle_count) {
                    r |= (1 << bit);
                }
                if (current_frame_coloured[1][current_row][col] <= cycle_count) {
                    g |= (1 << bit);
                }
                if (current_frame_coloured[2][current_row][col] <= cycle_count) {
                    b |= (1 << bit);
                }
                --bit;
            }

            matrix_sendLine(r,g,b);

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
	TCCR0B = ((1 << CS01) | (1 << CS00)); // (0b00000011)(3) clock prescalar to 64

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

    if (c == '\n') {
        if (rx_verbose) {
            USART_Transmit(c);
        }
    }

    if (rx_state == RX_COMMAND) {
        // reset the command arguments
        rx_args = 0;

        if (c == 'V') {
            // Send no responses.
            rx_verbose = 0;
        } else if (c == 'v') {
            // Send responses (default value).
            rx_verbose = 1;
        } else if (c == 'f' || c == 'p' || c == 'i' || c == 'c' || c == 's') {
            // Commands are only one byte.
            rx_cmd = c;
            // Now get the arguments.
            rx_state = RX_ARGS;

            if (rx_verbose) {
                USART_Transmit('*');
                USART_Transmit(rx_cmd);
                USART_Transmit('-');
            }
        }
    } else {
        // rx_state == RX_ARGS

        // Arguments are terminated by new line, unless sending image data.
        if (rx_cmd != 'i' && c == '\n') {

            // Newline so execute the command
            // 'f' is the command for font,
            // 'p' is for pattern,
            // 'c' is for display character
            // 's' is for scroll characters
            if (rx_cmd == 'f' || rx_cmd == 'p') {
                source_array = rx_cmd;
                source_index = rx_args;
                // Make it display now.
                frame_time = 0;
            } else if (rx_cmd == 'c') {
                source_array = 'f';
                source_index = rx_args;
                frame_time = 0;
            } else if (rx_cmd == 's') {
                source_array = 'F';
                frame_time = 0;
                // Set a blank screen to scroll into.
                //frame_SetMono_P(current_frame, font[0], font[0], font[0]);
                frame_Colourise_P(current_frame_coloured, font[0], example_colour);
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
                    if (rx_verbose) {
                        USART_Transmit(c);
                    }
                }
            } else if (rx_cmd == 'c' || rx_cmd == 's') {
                if (c > 0x19 && c < 0xff) {
                    rx_args = c - 32;
                    if (rx_verbose) {
                        USART_Transmit(c);
                    }

                    if (rx_cmd == 's') {
                        // Add to the scroll buffer.
                        scroll_Push(rx_args);
                    }

                }
            } else if (rx_cmd == 'i') {
                //USART_Transmit(c);
                if (rxBuildImage(image, c)) {
                    // Acknowledge the success.
                    if (rx_verbose) {
                        USART_Transmit('I');
                    }
                    source_array = 'i';
                    frame_SetColoured(current_frame_coloured, image[0], image[1], image[2]);
                    // Make it display now.
                    frame_time = 0;
                    rx_state = RX_COMMAND;
                }

            }
        }
    }

}

/**
 * State machine for building an image from serial input.
 *
 * Expects bytes (not ascii characters) RGBRGBRG...
 *
 */
uint8_t rxBuildImage(uint8_t image[][8][8], uint8_t c)
{
    static uint8_t state = 0;
    static uint8_t col = 0;
    static uint8_t row = 0;
    static uint8_t num = 0;


    // Build the number.
    /*
    if (c > 0x2F && c < 0x3A) {
        if (num > 0) {
            // Expecting decimal numbers like 24,
            // so for each new argument multiply by ten (shift left in decimal).
            num *= 10;
            num += c - 0x30;
        } else {
            num = c - 0x30;
        }

    }
*/
/*
    if (c == ',') { // Comma denotes next colour.
        //USART_Transmit(num);
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
    } else {
        num = c;
    }
*/

    num = c;
    if (state == 0) { // red
        image[0][row][col] = num;
    } else if (state == 1) { // green
        image[1][row][col] = num;
    } else if (state == 2) { // blue
        image[2][row][col] = num;
    }
    // Next colour
    state++;

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

    if (row > 7 || c == -1) {
        // Reset statics.
        row = 0;
        col = 0;
        state = 0;
        num = 0;

        // finished building
        return 1;
    }

    // Need more to finish the build
    return 0;
}

