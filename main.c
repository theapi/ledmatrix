//#define F_CPU 16000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define BAUD 115200
#include <util/setbaud.h>

#define PIN_DATA   PB3 // DS - MOSI - 11
#define PIN_CLOCK  PB5 // SH_CP - SCK - 13
#define PIN_LATCH  PB1 // ST_CP - 9
#define PIN_OE     PC3 // Output Enable - 17

// From Android.h
#define LSBFIRST 0
#define MSBFIRST 1

// 32 * 0.000004 = 0.000128 so ISR gets called every 128us
// 32 * 0.000004 * 8 = 0.001024 = about 1khz for whole matrix (NB zero based so register one less)
#define COMPARE_REG 31 // OCR0A when to interupt (datasheet: 14.9.4)
#define MILLIS_TICKS 8  // number of ISR calls before a millisecond is counted (ish)

#define T1 1 * MILLIS_TICKS // timeout value (mSec)
#define MAX_CYCLE_COUNT 16 // number of cycles needed brightness (a cycle is lighting each row once)

#define RX_BUFFER_LEN 64 // How many bytes the usart receive buffer can hold
#define TX_BUFFER_LEN 64 // How many bytes the usart send buffer can hold

#define SOURCE_SIZE_FONT    27 // The number of items in the font source array.
#define SOURCE_SIZE_PATTERN 3 // The number of items in the pattern source array.

/********************************************************************************
Function Prototypes
********************************************************************************/

void timerInit(void); //all the usual mcu stuff
void initSPI(void);
void USART_Init();
void USART_Transmit( unsigned char data );
void USART_rxProcess(void);
void setFrame(const uint8_t red[8], const uint8_t green[8], const uint8_t blue[8]);
void setFrame_P(const uint8_t red[8], const uint8_t green[8], const uint8_t blue[8]);
void setFrameBuffer(uint8_t red[8], uint8_t green[8], uint8_t blue[8]);
void frameBufferFlipV(void);
void frameBufferFlipH(void);
void frameFlipH(void);
void frameFlipV(void);
void latchLow(void);
void latchHigh(void);
void ledsDisable(void);
void ledsEnable(void);
void sendLine(uint8_t red, uint8_t blue, uint8_t green);
void sendByte(uint8_t bitOrder, uint8_t byte);
uint8_t bitReverse(uint8_t x);
void swap(uint8_t *px, uint8_t *py);



/********************************************************************************
Global Variables
********************************************************************************/

uint8_t cycle_count; // keeps track of the number of times a complete multiplex loop has happened.
uint8_t current_row; // Which row of the frame is currently being shown via the multiplexing.
uint8_t current_frame[3][8]; // The current frame being displayed
uint8_t framebuffer[3][8]; // the current cycle of the frame, seperated into RGB (the ones and zeros to send)
unsigned long current_frame_duration = 2000 * MILLIS_TICKS; // millis to show the current frame.

// States for the usart receive processing.
enum rx_states {RX_COMMAND, RX_ARGS};
enum rx_states rx_state = RX_COMMAND;
uint8_t rx_cmd;  // The command being requested by serial data.
uint8_t rx_args; // The argument for rx_cmd being requested by serial data.


uint8_t junk; // throwaway variable for shift register SPI return data

uint8_t source_array; // The array that is the source of data.
uint8_t source_index; // The source array index that is currently being shown.

// anode low = ON with pnp
uint8_t anodes[8] = {
  0b01111111,
  0b10111111,
  0b11011111,
  0b11101111,
  0b11110111,
  0b11111011,
  0b11111101,
  0b11111110,
};

/*
uint8_t pattern[1][8] = {
	{
	  0b00000000,
	  0b01100110,
	  0b01100110,
	  0b00000000,
	  0b01111110,
	  0b01111110,
	  0b00111100,
	  0b00000000,
	}
};
*/

const uint8_t pattern[SOURCE_SIZE_PATTERN][8] PROGMEM = {
    {0x00, 0x66, 0x66, 0x00, 0x42, 0x42, 0x3C, 0x00}, // :)
    {0x00, 0x00, 0x66, 0x00, 0x42, 0x42, 0x3C, 0x00}, // :)
    {0x00, 0x66, 0x66, 0x00, 0x7E, 0x7E, 0x3C, 0x00}, // :O
};


// https://github.com/dhepper/font8x8/blob/master/font8x8_basic.h
const uint8_t font[SOURCE_SIZE_FONT][8] PROGMEM = {
{ 0x3E, 0x63, 0x7B, 0x7B, 0x7B, 0x03, 0x1E, 0x00},   // U+0040 (@)
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


volatile unsigned int time1;
volatile unsigned int frame_time;
volatile uint8_t data_sent; // whether the data has been sent and just needs to be latched.

volatile unsigned char rx_buffer[RX_BUFFER_LEN];
volatile unsigned char rx_head;
unsigned char rx_tail; // The last buffer byte processed

volatile unsigned char tx_buffer[TX_BUFFER_LEN];
volatile unsigned char tx_head;
volatile unsigned char tx_tail;

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
    ledsEnable();
}

/**
 * Interrupt when the USART receives a byte.
 */
ISR(USART_RX_vect)
{
   // Add to the receive buffer
   rx_buffer[rx_head] = UDR0;

   // Increment the index
   rx_head++;
   if (rx_head >= RX_BUFFER_LEN) {
       rx_head = 0;
   }
}

/**
 * Interrupt when the USART is ready to send more bytes.
 */
ISR(USART_UDRE_vect)
{
    // If head & tail are not in sync, send the next byte in byte.
    if (tx_head != tx_tail) {
        UDR0 = tx_buffer[tx_tail];
        // Increment the tail index
        tx_tail++;
        if (tx_tail >= TX_BUFFER_LEN) {
            tx_tail = 0;
        }
    } else {
        // Nothing left to send so turn off this interrupt
        UCSR0B &= ~(1 << UDRIE0);
    }
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

	uint8_t frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	setFrame(frame, frame, frame);
    setFrameBuffer(frame, frame, frame);

	initSPI();
	USART_Init();
	timerInit();

	ledsEnable(); // leds on

	data_sent = 0;

	// crank up the ISRs
	sei();

	// main loop
    while(1) {

        // Handle unprocessed received serial data.
        USART_rxProcess();

    	if (time1 == 0) {
    		// reset the timer
    		time1 = T1;

    	}

    	// check for countdown reached 0
    	if (frame_time == 0) {
    		// reset the frame timer
    		frame_time = current_frame_duration;

    		if (source_array == 'f') {
    		    if (source_index >= SOURCE_SIZE_FONT) {
                    source_index = 0;
                }
    		    setFrame_P(font[source_index], font[source_index], font[source_index]);
    		} else {
    		    if (source_index >= SOURCE_SIZE_PATTERN) {
    		        source_index = 0;
    		    }
    		    setFrame_P(pattern[source_index], pattern[source_index], pattern[source_index]);
    		    source_index++;
    		}

    		// @todo Mmm, not right that the framebuffer needs to be set BEFORE frameFlip()
    		// must be a pointer/reference problem...
    		setFrameBuffer(current_frame[0], current_frame[1], current_frame[2]);

			// Currently my matrix is upside down (pins are on the bottom - I have no mount)
			// so fix the frame on the fly.
    		// This takes too much time for an OCR0A of 31 & bit banging (SPI is ok)
			//frameFlipH();
			//frameFlipV();

			//setFrameBuffer(current_frame[0], current_frame[1], current_frame[2]);

    	}

    	if (!data_sent) {
    		// Send the next line ready to be latched in the ISR
    		sendLine(
    			framebuffer[0][current_row], // Red
				framebuffer[1][current_row], // Green
				framebuffer[2][current_row]  // Blue
    		);

			// Flag that the data is ready to be latched in the ISR.
    		data_sent = 1;
    	}

    }
}


/********************************************************************************
Functions
********************************************************************************/

/**
 * Flip the frame  on the horizontal axis.
 */
void frameFlipH(void)
{
	uint8_t i;

	for (i = 0; i < 3; i++) {
		swap(&current_frame[i][0], &current_frame[i][7]);
		swap(&current_frame[i][1], &current_frame[i][6]);
		swap(&current_frame[i][2], &current_frame[i][5]);
		swap(&current_frame[i][3], &current_frame[i][4]);
	}

}

/**
 * Flip the frame  on the v axis.
 */
void frameFlipV(void)
{
	uint8_t i;
	for (i = 0; i < 8; i++) {
		// red
		current_frame[0][i] = bitReverse(current_frame[0][i]);
        // green
		current_frame[1][i] = bitReverse(current_frame[1][i]);
		// blue
		current_frame[2][i] = bitReverse(current_frame[2][i]);
	}

}

/**
 * Flip the frame buffer on the horizontal axis.
 */
void frameBufferFlipH(void)
{
	uint8_t i;

	for (i = 0; i < 3; i++) {
		swap(&framebuffer[i][0], &framebuffer[i][7]);
		swap(&framebuffer[i][1], &framebuffer[i][6]);
		swap(&framebuffer[i][2], &framebuffer[i][5]);
		swap(&framebuffer[i][3], &framebuffer[i][4]);
	}

}

void swap(uint8_t *px, uint8_t *py)
{
	uint8_t temp;

	temp = *px;
	*px = *py;
	*py = temp;
}

/**
 * Flip the frame buffer on the vertical axis.
 */
void frameBufferFlipV(void)
{
	uint8_t i;
	for (i = 0; i < 8; i++) {
		// red
		framebuffer[0][i] = bitReverse(framebuffer[0][i]);
        // green
		framebuffer[1][i] = bitReverse(framebuffer[1][i]);
		// blue
		framebuffer[2][i] = bitReverse(framebuffer[2][i]);
	}

}

/**
 * Sets the ones & zeros to be sent to the display
 */
void setFrameBuffer(uint8_t red[8], uint8_t green[8], uint8_t blue[8])
{
  uint8_t i;
  for (i = 0; i < 8; i++) {
    framebuffer[0][i] = red[i];
    framebuffer[1][i] = green[i];
    framebuffer[2][i] = blue[i];
  }
}

/**
 * Stores the data for the frames so the bytes are hex values of the brightness.
 */
void setFrame(const uint8_t red[8], const uint8_t green[8], const uint8_t blue[8])
{
  uint8_t i;
  for (i = 0; i < 8; i++) {
    current_frame[0][i] = red[i];
    current_frame[1][i] = green[i];
    current_frame[2][i] = blue[i];
  }
}

/**
 * Stores the data from PROGMEM for the frames so the bytes are hex values of the brightness.
 */
void setFrame_P(const uint8_t red[8], const uint8_t green[8], const uint8_t blue[8])
{
  uint8_t i;
  for (i = 0; i < 8; i++) {
    current_frame[0][i] = pgm_read_byte(red + i);
    current_frame[1][i] = pgm_read_byte(green + i);
    current_frame[2][i] = pgm_read_byte(blue + i);
  }
}

/**
 * Make least significant bit highest etc.
 *
 * 11001100 -> 00110011
 */
uint8_t bitReverse(uint8_t x)
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

/**
 * Set the latch low.
 */
void latchLow(void)
{
	PORTB &= ~(1 << PIN_LATCH);
}

/**
 * Take the latch pin high, to move the shifted data into position
 */
void latchHigh(void)
{
	PORTB |= (1 << PIN_LATCH);
}

/**
 * OE low to turn on the leds.
 */
void ledsEnable(void)
{
	PORTC &= ~(1 << PIN_OE);
}

/**
 * Turn off the leds with OE high.
 *
 * To prevent ghosting/leakage.
 * The process seems to need time to settle (parasitic capcitance?)
 */
void ledsDisable(void)
{
	PORTC |= (1 << PIN_OE);
}

/**
 * Send out the next line of prepared data.
 */
void sendLine(uint8_t red, uint8_t green, uint8_t blue)
{
	// NOT (invert) the bytes so the patterns are 1 == ON 0 == OFF (common anode)
    if (cycle_count < 5) {
        sendByte(MSBFIRST, ~red);
    } else {
        sendByte(MSBFIRST, ~0);
    }

    if (cycle_count < 12) {
        sendByte(MSBFIRST, ~blue);
    } else {
        sendByte(MSBFIRST, ~0);
    }

	// YEP, the hardware I built needs the green to be least significate bit first :(
    if (cycle_count < 16) {
        sendByte(LSBFIRST, ~green);
	} else {
        sendByte(LSBFIRST, ~0);
    }

	// ANODES
	sendByte(MSBFIRST, anodes[current_row]);

	++current_row;
	if (current_row > 7) {
		current_row = 0;

		// One cycle completed.
		++cycle_count;
		if (cycle_count >= MAX_CYCLE_COUNT) {
		    cycle_count = 0;
		}

	}
}

void sendByte(uint8_t bitOrder, uint8_t byte)
{
/*
    // Bit bang
	uint8_t i;
	uint8_t bit;

	for (i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST) {
			bit = i;
		} else {
			bit = 7 - i;
		}

		if (byte & (1 << bit)) {
			PORTB |= (1 << PIN_DATA); // HIGH
		} else {
			PORTB &= ~(1 << PIN_DATA); // LOW
		}

		// Clock in the bit
		PORTB |= (1 << PIN_CLOCK); // HIGH
		PORTB &= ~(1 << PIN_CLOCK); // LOW
	}
*/

	// SPI
	if (bitOrder == LSBFIRST) {
		SPCR |= (1 << DORD); // high
	} else {
		SPCR &= ~(1 << DORD); // low
	}

	// Send the byte
	SPDR = byte;
	// Wait for SPI process to finish
	while(!(SPSR & (1<<SPIF)));
	// Need to read the result
	junk = SPDR;

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

void USART_Init(void)
{
    //UBRR0H = (UBRR >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
    //UBRR0L = UBRR;        // Load lower 8-bits of the baud rate value into the low byte of the UBRR register

    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    #if USE_2X
        UCSR0A |= (1 << U2X0);
    #else
        UCSR0A &= ~(1 << U2X0);
    #endif

    // Use 8-bit character sizes & 1 stop bit
    UCSR0C = (0 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01);

    // Enable receiver, transmitter and interrupt on receive.
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
}

void USART_Transmit( unsigned char data )
{
    // Buffer the byte to be sent by the ISR
    tx_buffer[tx_head] = data;
    // Increment the head index
    tx_head++;
    if (tx_head >= TX_BUFFER_LEN) {
        tx_head = 0;
    }

    // Ensure the interrupt to send this is on.
    UCSR0B |= (1 << UDRIE0);

    /*
    // Wait for empty transmit buffer
    while ( !( UCSR0A & (1 << UDRE0)) )
        ;
    // Put data into buffer, sends the data
    UDR0 = data;
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
void USART_rxProcess(void)
{
    if (rx_head == rx_tail) {
        // Nothing to do.
        return;
    }

    uint8_t c = rx_buffer[rx_tail];
    // Increase the processed index
    rx_tail++;
    if (rx_tail >= RX_BUFFER_LEN) {
        rx_tail = 0;
    }

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

