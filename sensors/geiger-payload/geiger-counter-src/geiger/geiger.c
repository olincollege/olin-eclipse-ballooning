/*
	Title: Geiger Counter with Serial Data Reporting
	Description: This is the firmware for the mightyohm.com Geiger Counter.
		There is more information at http://mightyohm.com/geiger 
		
	Author:		Jeff Keyzer
	Company:	MightyOhm Engineering
	Website:	http://mightyohm.com/
	Contact:	jeff <at> mightyohm.com
  
	This firmware controls the ATtiny2313 AVR microcontroller on board the Geiger Counter kit.
	
	When an impulse from the GM tube is detected, the firmware flashes the LED and produces a short
	beep on the piezo speaker.  It also outputs an active-high pulse (default 100us) on the PULSE pin.
	
	A pushbutton on the PCB can be used to mute the beep.
	
	A running average of the detected counts per second (CPS), counts per minute (CPM), and equivalent dose
	(uSv/hr) is output on the serial port once per second. The dose is based on information collected from 
	the web, and may not be accurate.
	
	The serial port is configured for BAUD baud, 8-N-1 (default 9600).
	
	The data is reported in comma separated value (CSV) format:
	CPS, #####, CPM, #####, uSv/hr, ###.##, SLOW|FAST|INST
	
	There are three modes.  Normally, the sample period is LONG_PERIOD (default 60 seconds). This is SLOW averaging mode.
	If the last five measured counts exceed a preset threshold, the sample period switches to SHORT_PERIOD seconds (default 5 seconds).
	This is FAST mode, and is more responsive but less accurate. Finally, if CPS > 255, we report CPS*60 and switch to
	INST mode, since we can't store data in the (8-bit) sample buffer.  This behavior could be customized to suit a particular
	logging application.
	
	The largest CPS value that can be displayed is 65535, but the largest value that can be stored in the sample buffer is 255.
	
	***** WARNING *****
	This Geiger Counter kit is for EDUCATIONAL PURPOSES ONLY.  Don't even think about using it to monitor radiation in
	life-threatening situations, or in any environment where you may expose yourself to dangerous levels of radiation.
	Don't rely on the collected data to be an accurate measure of radiation exposure! Be safe!
	
	
	Change log:
	8/4/11 1.00: Initial release for Chaos Camp 2011!


		Copyright 2011 Jeff Keyzer, MightyOhm Engineering
 
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Includes
#include <avr/io.h>			// this contains the AVR IO port definitions
#include <avr/interrupt.h>	// interrupt service routines
#include <avr/pgmspace.h>	// tools used to store variables in program memory
#include <avr/sleep.h>		// sleep mode utilities
#include <util/delay.h>		// some convenient delay functions
#include <stdlib.h>			// some handy functions like utoa()

// Defines
#define VERSION			"1.00"
#define URL				"http://mightyohm.com/geiger"

#define	F_CPU			8000000	// AVR clock speed in Hz
#define	BAUD			9600	// Serial BAUD rate
#define SER_BUFF_LEN	11		// Serial buffer length
#define THRESHOLD		1000	// CPM threshold for fast avg mode
#define LONG_PERIOD		60		// # of samples to keep in memory in slow avg mode
#define SHORT_PERIOD	5		// # or samples for fast avg mode
#define SCALE_FACTOR	57		//	CPM to uSv/hr conversion factor (x10,000 to avoid float)
#define PULSEWIDTH		100		// width of the PULSE output (in microseconds)

// Function prototypes
void uart_putchar(char c);			// send a character to the serial port
void uart_putstring(char *buffer);		// send a null-terminated string in SRAM to the serial port
void uart_putstring_P(char *buffer);	// send a null-terminated string in PROGMEM to the serial port

void checkevent(void);	// flash LED and beep the piezo
void sendreport(void);	// log data over the serial port

// Global variables
volatile uint8_t nobeep;		// flag used to mute beeper
volatile uint16_t count;		// number of GM events that has occurred
volatile uint16_t slowcpm;		// GM counts per minute in slow mode
volatile uint16_t fastcpm;		// GM counts per minute in fast mode
volatile uint16_t cps;			// GM counts per second, updated once a second
volatile uint8_t overflow;		// overflow flag

volatile uint8_t buffer[LONG_PERIOD];	// the sample buffer
volatile uint8_t idx;					// sample buffer index

volatile uint8_t eventflag;	// flag for ISR to tell main loop if a GM event has occurred
volatile uint8_t tick;		// flag that tells main() when 1 second has passed

char serbuf[SER_BUFF_LEN];	// serial buffer
uint8_t mode;				// logging mode, 0 = slow, 1 = fast, 2 = inst


// Interrupt service routines

//	Pin change interrupt for pin INT0
//	This interrupt is called on the falling edge of a GM pulse.
ISR(INT0_vect)
{
	if (count < UINT16_MAX)	// check for overflow, if we do overflow just cap the counts at max possible
		count++; // increase event counter

	// send a pulse to the PULSE connector
	// a delay of 100us limits the max CPS to about 8000
	// you can comment out this code and increase the max CPS possible (up to 65535!)
	PORTD |= _BV(PD6);	// set PULSE output high
	_delay_us(PULSEWIDTH);
	PORTD &= ~(_BV(PD6));	// set pulse output low
		
	eventflag = 1;	// tell main program loop that a GM pulse has occurred
}

//	Pin change interrupt for pin INT1 (pushbutton)
//	If the user pushes the button, this interrupt is executed.
//	We need to be careful about switch bounce, which will make the interrupt
//	execute multiple times if we're not careful.
ISR(INT1_vect)
{
	_delay_ms(25);					// slow down interrupt calls (crude debounce)
	
	if ((PIND & _BV(PD3)) == 0)		// is button still pressed?
		nobeep ^= 1;				// toggle mute mode
	
	EIFR |= _BV(INTF1);				// clear interrupt flag to avoid executing ISR again due to switch bounce
}

/*	Timer1 compare interrupt 
 *	This interrupt is called every time TCNT1 reaches OCR1A and is reset back to 0 (CTC mode).
 *  Timer1 is setup so this happens once a second.
 */
ISR(TIMER1_COMPA_vect)
{
	uint8_t i;	// index for fast mode
	tick = 1;	// update flag
	
	//PORTB ^= _BV(PB4);	// toggle the LED (for debugging purposes)
	cps = count;
	slowcpm -= buffer[idx];		// subtract oldest sample in sample buffer
	
	if (count > UINT8_MAX) {	// watch out for overflowing the sample buffer
		count = UINT8_MAX;
		overflow = 1;
	}
			
	slowcpm += count;			// add current sample
	buffer[idx] = count;	// save current sample to buffer (replacing old value)
	
	// Compute CPM based on the last SHORT_PERIOD samples
	fastcpm = 0;
	for(i=0; i<SHORT_PERIOD;i++) {
		int8_t x = idx - i;
		if (x < 0)
			x = LONG_PERIOD + x;
		fastcpm += buffer[x];	// sum up the last 5 CPS values
	}
	fastcpm = fastcpm * (LONG_PERIOD/SHORT_PERIOD);	// convert to CPM
	
	// Move to the next entry in the sample buffer
	idx++;
	if (idx >= LONG_PERIOD)
		idx = 0;
	count = 0;  // reset counter
}

// Functions

// Send a character to the UART
void uart_putchar(char c)
{
	if (c == '\n') uart_putchar('\r');	// Windows-style CRLF
  
	loop_until_bit_is_set(UCSRA, UDRE);	// wait until UART is ready to accept a new character
	UDR = c;							// send 1 character
}

// Send a string in SRAM to the UART
void uart_putstring(char *buffer)	
{	
	// start sending characters over the serial port until we reach the end of the string
	while (*buffer != '\0') {	// are we at the end of the string yet?
		uart_putchar(*buffer);	// send the contents
		buffer++;				// advance to next char in buffer
	}
}

// Send a string in PROGMEM to the UART
void uart_putstring_P(char *buffer)	
{	
	// start sending characters over the serial port until we reach the end of the string
	while (pgm_read_byte(buffer) != '\0')	// are we done yet?
		uart_putchar(pgm_read_byte(buffer++));	// read byte from PROGMEM and send it
}

// flash LED and beep the piezo
void checkevent(void)
{
	if (eventflag) {		// a GM event has occurred, do something about it!
		eventflag = 0;		// reset flag as soon as possible, in case another ISR is called while we're busy

		PORTB |= _BV(PB4);	// turn on the LED
		
		if(!nobeep) {		// check if we're in mute mode
			TCCR0A |= _BV(COM0A0);	// enable OCR0A output on pin PB2
			TCCR0B |= _BV(CS01);	// set prescaler to clk/8 (1Mhz) or 1us/count
			OCR0A = 160;	// 160 = toggle OCR0A every 160ms, period = 320us, freq= 3.125kHz
		}
		
		// 10ms delay gives a nice short flash and 'click' on the piezo
		_delay_ms(10);	
			
		PORTB &= ~(_BV(PB4));	// turn off the LED
		
		TCCR0B = 0;				// disable Timer0 since we're no longer using it
		TCCR0A &= ~(_BV(COM0A0));	// disconnect OCR0A from Timer0, this avoids occasional HVPS whine after beep
	}	
}
// log data over the serial port
void sendreport(void)
{
	uint32_t cpm;	// This is the CPM value we will report
	if(tick) {	// 1 second has passed, time to report data via UART
		tick = 0;	// reset flag for the next interval
			
		if (overflow) {
			cpm = cps*60UL;
			mode = 2;
			overflow = 0;
		}				
		else if (fastcpm > THRESHOLD) {	// if cpm is too high, use the short term average instead
			mode = 1;
			cpm = fastcpm;	// report cpm based on last 5 samples
		} else {
			mode = 0;
			cpm = slowcpm;	// report cpm based on last 60 samples
		}
		
		// Send CPM value to the serial port
		uart_putstring_P(PSTR("CPS, "));
		utoa(cps, serbuf, 10);		// radix 10
		uart_putstring(serbuf);
			
		uart_putstring_P(PSTR(", CPM, "));
		ultoa(cpm, serbuf, 10);		// radix 10
		uart_putstring(serbuf);
			
		uart_putstring_P(PSTR(", uSv/hr, "));
	
		// calculate uSv/hr based on scaling factor, and multiply result by 100
		// so we can easily separate the integer and fractional components (2 decimal places)
		uint32_t usv_scaled = (uint32_t)(cpm*SCALE_FACTOR);	// scale and truncate the integer part
			
		// this reports the integer part
		utoa((uint16_t)(usv_scaled/10000), serbuf, 10);	
		uart_putstring(serbuf);
			
		uart_putchar('.');
			
		// this reports the fractional part (2 decimal places)
		uint8_t fraction = (usv_scaled/100)%100;
		if (fraction < 10)
			uart_putchar('0');	// zero padding for <0.10
		utoa(fraction, serbuf, 10);
		uart_putstring(serbuf);
			
		// Tell us what averaging method is being used
		if (mode == 2) {
			uart_putstring_P(PSTR(", INST"));
		} else if (mode == 1) {
			uart_putstring_P(PSTR(", FAST"));
		} else {
			uart_putstring_P(PSTR(", SLOW"));
		}			
			
		// We're done reporting data, output a newline.
		uart_putchar('\n');	
	}	
}

// Start of main program
int main(void)
{	
	// Configure the UART	
	// Set baud rate generator based on F_CPU
	UBRRH = (unsigned char)(F_CPU/(16UL*BAUD)-1)>>8;
	UBRRL = (unsigned char)(F_CPU/(16UL*BAUD)-1);
	
	// Enable USART transmitter and receiver
	UCSRB = (1<<RXEN) | (1<<TXEN);

	uart_putstring_P(PSTR("mightyohm.com Geiger Counter " VERSION "\n"));
	uart_putstring_P(PSTR(URL "\n"));

	// Set up AVR IO ports
	DDRB = _BV(PB4) | _BV(PB2);  // set pins connected to LED and piezo as outputs
	DDRD = _BV(PD6);	// configure PULSE output
	PORTD |= _BV(PD3);	// enable internal pull up resistor on pin connected to button
	
	// Set up external interrupts	
	// INT0 is triggered by a GM impulse
	// INT1 is triggered by pushing the button
	MCUCR |= _BV(ISC01) | _BV(ISC11);	// Config interrupts on falling edge of INT0 and INT1
	GIMSK |= _BV(INT0) | _BV(INT1);		// Enable external interrupts on pins INT0 and INT1
	
	// Configure the Timers		
	// Set up Timer0 for tone generation
	// Toggle OC0A (pin PB2) on compare match and set timer to CTC mode
	TCCR0A = (0<<COM0A1) | (1<<COM0A0) | (0<<WGM02) |  (1<<WGM01) | (0<<WGM00);
	TCCR0B = 0;	// stop Timer0 (no sound)

	// Set up Timer1 for 1 second interrupts
	TCCR1B = _BV(WGM12) | _BV(CS12);  // CTC mode, prescaler = 256 (32us ticks)
	OCR1A = 31250;	// 32us * 31250 = 1 sec
	TIMSK = _BV(OCIE1A);  // Timer1 overflow interrupt enable
	
	sei();	// Enable interrupts

	while(1) {	// loop forever
		
		// Configure AVR for sleep, this saves a couple mA when idle
		set_sleep_mode(SLEEP_MODE_IDLE);	// CPU will go to sleep but peripherals keep running
		sleep_enable();		// enable sleep
		sleep_cpu();		// put the core to sleep
		
		// Zzzzzzz...	CPU is sleeping!
		// Execution will resume here when the CPU wakes up.
		
		sleep_disable();	// disable sleep so we don't accidentally go to sleep
		
		checkevent();	// check if we should signal an event (led + beep)
	
		sendreport();	// send a log report over serial
		
		checkevent();	// check again before going to sleep
		
	}	
	return 0;	// never reached
}

