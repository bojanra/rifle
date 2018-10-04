/*****************************************************************************
  Rifle - antenna rotator interface for YAESU G-1000DXC series rotator

  -----------------------------------------------------------------------------

Basically all work is build around the timer wich is 4 times the baudrate and
implements a software uart. The target is ATTINY861.

In normal mode the Led is blinking Duty Cyle 10%.

Serial connection is 9600 8n1

When more than (120 seconds) DEADTIME after each start of movement the target is not reached
the movement is stopped.

The watchdog is active via the main loop.

Commands are inside the program.

*****************************************************************************/
// internal clock
#define F_CPU 8000000
#define TIMER0PRESET 83         // preset for timer 0 - responsible for sampling rate -> baudrate
#define DEADTIME 120            // after DEADTIME seconds since start of movement the rotator is stopped

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

const char version[] PROGMEM = "Rifle V5.3 full\n";
const char help[] PROGMEM = "# Command list\n\
## YAESU GS232\n\
B - get only elevation +0eee\n\
C - get only azimut +0aaa\n\
C2 - get readings in form +0aaa+0eee\n\
S - stop motion & tuning mode & debug output (space)\n\
Maaa - set destination azimut to aaa\n\
Waaa eee - set destination aaa azimut and eee elevation\n\
L - start rotating left\n\
R - start rotating right\n\
U - start moving up\n\
D - start moving down\n\
A - stop azimut rotation\n\
E - stop elevation moving\n\
\n\
(no command termination - direct reading)\n\
t - tuning mode on - continously send readings like C2\n\
T - stop tuning mode\n\
r - reset via watchdog\n\
v - Version string\n\
h - Help - this command list\n";

#define	TX_PIN      PA7
#define RX_PIN      PA6
#define LED_PIN     PA3
#define MOSL        PA1   // input for received fieldstrength

#define LEFT_PIN    PB0   // rotator A - azimuth
#define RIGHT_PIN   PB1
#define SPEEDCTRL_A PB3
#define BEAR_A      PA0   // input for rotator A bearing

#define DOWN_PIN    PA4   // rotator B - elevation 0-90
#define UP_PIN      PA5
#define SPEEDCTRL_B PB6
#define BEAR_B      PA2

#define ADC_AZIMUTH   0   // read bearing from port PA0
#define ADC_ELEVATION 2   // read bearing from port PA2

static volatile uint8_t   seconds;
static volatile uint8_t   reset;
static volatile uint8_t   job;
static volatile uint8_t   rotator_status;
static volatile uint16_t  target_a;
static volatile uint16_t  target_b;
static volatile uint16_t  bearing_a;
static volatile uint16_t  bearing_b;
static volatile uint16_t  mosley;
static volatile uint16_t  differ;

// job codes
#define job_ready         0
#define job_getversion    1
#define job_help          2
#define job_stop_motion   4
#define job_get_reading   8
#define job_tuning        16
#define job_get_azimut    32
#define job_get_elevation 64
#define job_get_help      128

// status codes
#define moving_a    1
#define moving_b    2

// clock times
static volatile uint16_t ticks;			// 192 of these make 1/50th of a second
static volatile uint8_t fiftieths;

// software uart registers
static uint8_t	uart_rxd;		        // the byte being received
static volatile uint8_t	uart_txd;		// the byte being transmitted
static uint8_t	uart_txtick;	      // tick count for the transmitter
static uint8_t	uart_rxtick;	      // tick count for the receiver
static uint8_t	uart_txbit;		      // bit count for tx
static uint8_t	uart_rxbit;		      // bit count for rx
static volatile uint8_t	uart_status;// uart status
static volatile uint8_t next_write;	// the pointers to the receive2 buffer

static unsigned char buffer[16]; // circular buffer for receiver

// status bits
#define txbusy 0				// set if a byte is in transmission
#define rxbusy 1				// set if a byte is being received
#define rxframe 2				// set if the stop byte isn't a one

#define STOP_AZIMUT PORTB &= ~(1<<RIGHT_PIN | 1<<LEFT_PIN);
#define GO_RIGHT    STOP_AZIMUT; PORTB |= (1<<RIGHT_PIN);
#define GO_LEFT     STOP_AZIMUT; PORTB |= (1<<LEFT_PIN);
#define STOP_ELEVATION PORTA &= ~( 1<<UP_PIN | 1<<DOWN_PIN);
#define GO_UP       STOP_ELEVATION; PORTA |= (1<<UP_PIN);
#define GO_DOWN     STOP_ELEVATION; PORTA |= (1<<DOWN_PIN);

ISR( TIMER0_OVF_vect) {
  // the interrupt signal

  // we arrive here four times per baud rate
  // preset the timer0
  TCNT0L = TIMER0PRESET;				// preset for timer

  // we first send the output signal, if there's anything to send,
  // since it needs to be somewhere close to accurate...
  // then we read the incoming signal, if there is one,
  // and finally we update the nearly-real-time clock
  // (we never display the clock but it timestamps the output data)

  if( uart_status & (1<<txbusy) ) {
    // we're transmitting something
    // increment the tick - we only do things every four ticks
    uart_txtick++;

    if( uart_txtick == 4) {
      // okay, we've work to do
      uart_txtick = 0;

      // is it the start bit?
      if( uart_txbit == 0) {
        // yes...
        PORTA &= ~(1<<TX_PIN);		// clear the start bit output
        uart_txbit++;
      }
      else {
        if( uart_txbit != 9) {
          // deal with the data bits
          if( uart_txd & 1)		// low bit set?
            PORTA |= (1<<TX_PIN);	// then set the data stream bit
          else
            PORTA &= ~(1<<TX_PIN);  // or clear, as required
          uart_txbit++;				// increment the bit count

          // and shift the data right
          uart_txd /= 2;
        }
        else {
          // deal with the stop bit
          PORTA |= (1<<TX_PIN);
          uart_txbit++;
        }
      }

      // and finally, if txbit is more than 9, we've done
      if( uart_txbit > 9) {
        uart_txbit = 0;		    		// clear the bit counter
        uart_status &= ~(1<<txbusy);  // and the busy status
      }
    }
  }


  // we may be receiving something
  // if we're *not* yet receiving, we check every clock tick to see if the input
  // line has gone into a stop bit
  // if it has, we wait for half a bit period and then sample every four ticks
  // to put together the rx data

  if( (uart_status & (1<<rxbusy)) == 0) {
    // we're idling
    // check to see if there's a start
    if( (PINA & (1<<RX_PIN)) == 0) {
      // we found a start bit!
      // set the tick count to 2, so we get the sample near the middle of the bit
      uart_rxtick = 2;
      // and flag that we're now busy
      uart_status |= (1<<rxbusy);
      // we're expecting the start bit now...
      uart_rxbit = 0;
      // and carry on with life
    }
  }
  else {
    // we're receiving something
    // inc the tick count
    uart_rxtick++;
    if( uart_rxtick == 4) {
      // we only sample when the tick = 0
      uart_rxtick = 0;

      // what we do depends on the bit count
      // 0 = start,
      // 1-8 = data
      // 9 = stop
      if (uart_rxbit == 0) {
        // start bit
        // it had better be 0 or it was a line glitch
        if ((PINA & (1<<RX_PIN)) == 0) {
          // it's a real start bit (probably) so deal with it
          // next bit will be data
          uart_rxbit ++;
          uart_rxd = 0;
        }
        else {
          // bad start bit, flag us back as not busy
          uart_status |= ~((1<<rxbusy));
        }
      }
      else {
        if (uart_rxbit < 9) {
          // data bit
          // shift the received bits down
          uart_rxd >>= 1;
          // and set the higest bit
          // if the data bit is a one, set we add the bit value to the rxd value
          if ((PINA & (1<<RX_PIN)) != 0) {
            uart_rxd |= (1<<7);
          }
          uart_rxbit ++;
        }
        else if (uart_rxbit == 9) {
          // stop bit
          // we're going to assume it's a valid bit, though we could check for
          // framing error here, and simply use this bit to wait for the first
          // stop bit period
          //     uart_rxbit++;
          // }
          // else {

          // finished, ready to start again
          uart_status &= ~(1<<rxbusy);
          // TEST THE RECEIVED CHARACTER
          if( uart_rxd == 'v') {
            job |= job_getversion;
            next_write = 0;
          }
          else if( uart_rxd == 'T') {
            job &= ~job_tuning;
            next_write = 0;
          }
          else if( uart_rxd == 'h' ) {
            job |= job_get_help;
          }
          else if( uart_rxd == 'r' ) {
            // reset
            while( 1) {
            }
          }
          else if( uart_rxd == 't' ) {
            job |= job_tuning;
            next_write = 0;
          }
          else if( uart_rxd == 13) {
            if( next_write == 8 && buffer[0] == 'W' && buffer[4] == ' ') {
              // command W
              target_a = 100*(buffer[1] & 0x0f)+10*(buffer[2] & 0x0f)+(buffer[3] & 0x0f);
              if( target_a <= 450) {
                // target is o.k. start moving //
                if( target_a > bearing_a) {
                  GO_RIGHT;
                }
                else if( target_a < bearing_a) {
                  GO_LEFT;
                }
                seconds = 0;
                rotator_status |= moving_a;
              }
              else {
                // stop moving //
                rotator_status &= ~moving_a;
                STOP_AZIMUT;
              }
              target_b = 100*(buffer[5] & 0x0f)+10*(buffer[6] & 0x0f)+(buffer[7] & 0x0f);
              if( target_b <= 150) {
                // target is o.k. start moving //
                if( target_b > bearing_b) {
                  GO_UP;
                }
                else if( target_b < bearing_b) {
                  GO_DOWN;
                }
                seconds = 0;
                rotator_status |= moving_b;
              }
              else {
                // stop moving //
                rotator_status &= ~moving_b;
                STOP_ELEVATION;
              }
            }
            else if( next_write == 4 && buffer[0] == 'M') {
              // command M
              target_a = 100*(buffer[1] & 0x0f)+10*(buffer[2] & 0x0f)+(buffer[3] & 0x0f);
              if( target_a <= 450) {
                // target is o.k. start moving //
                if( target_a > bearing_a) {
                  GO_RIGHT;
                }
                else if( target_a < bearing_a) {
                  GO_LEFT;
                }
                seconds = 0;
                rotator_status |= moving_a;
              }
              else {
                // stop moving //
                rotator_status &= ~moving_a;
                STOP_AZIMUT;
              }
            }
          else if( next_write == 1 && buffer[0] == 'S') {
              // command S - stop all
              rotator_status = 0;
              job = 0;
              STOP_ELEVATION;
              STOP_AZIMUT;
            }
            else if( next_write == 2 && buffer[0] == 'C' && buffer[1] == '2') {
              job |= job_get_reading;
            }
            else if( next_write == 1 && buffer[0] == 'C') {
              job |= job_get_azimut;
            }
            else if( next_write == 1 && buffer[0] == 'H' ) {
              job |= job_get_help;
            }
            else if( next_write == 1 && buffer[0] == 'B') {
              job |= job_get_elevation;
            }
            else if( next_write == 1 && buffer[0] == 'R') {
              GO_RIGHT;
            }
            else if( next_write == 1 && buffer[0] == 'L') {
              GO_LEFT;
            }
            else if( next_write == 1 && buffer[0] == 'U') {
              GO_UP;
            }
            else if( next_write == 1 && buffer[0] == 'D') {
              GO_DOWN;
            }
            else if( next_write == 1 && buffer[0] == 'A') {
              STOP_AZIMUT;
            }
            else if( next_write == 1 && buffer[0] == 'E') {
              STOP_ELEVATION;
            }
            else {
              // write debug info
              job |= job_getversion;
            }
            next_write = 0;
          }
          else {
            // store the data into the buffer
            buffer[next_write++] = uart_rxd;
            // buffer protection
            next_write &= 0x0f;
          }
        }
      }
    }
  }

  // increment the clock
  ticks++;
  if( ticks == 768) {
    // run this code every 20ms
    ticks = 0;
    fiftieths++;

    // ADC stuff
    if( (ADCSRA & (1<<ADSC)) == 0) {
      if( ADMUX == 0) {
        // read bearing a - azimuth
        bearing_a = ADCW;
        bearing_a >>= 1;
        ADMUX = ADC_ELEVATION;
      }
      else {
        // read bearing b - elevation
        bearing_b = ADCW;
        bearing_b >>= 2;   // and divide by 2
        if( bearing_b >= 90)
          bearing_b -= 90; // offset because of "incorrect" mounting of unit
        else
          bearing_b = 0;
        ADMUX = ADC_AZIMUTH;
      }
      ADCSRA |= (1<<ADSC);        // start AD conversion
    }

    // movement control - do not overshoot - only check if moving
    if( (fiftieths % 10 == 0) && rotator_status) {
      // we have arrived at the target bearing
      if( target_a == bearing_a ||
          ((PORTB & (1<<LEFT_PIN)) && target_a > bearing_a) ||
          ((PORTB & (1<<RIGHT_PIN)) && target_a < bearing_a)) {
        rotator_status &= ~moving_a;
        STOP_AZIMUT;
      }
      if( target_b == bearing_b ||
          ((PORTA & (1<<DOWN_PIN)) && target_b > bearing_b) ||
          ((PORTA & (1<<UP_PIN)) && target_b < bearing_b)) {
        rotator_status &= ~moving_b;
        STOP_ELEVATION;
      }
    }

    if( fiftieths == 5 || fiftieths == 15 || fiftieths == 30) {
      // turn led off
      PORTA |= (1<<LED_PIN);
    }

    // when moving do additional blink
    if( fiftieths == 25 && rotator_status) {
      // turn led on
      PORTA &= ~(1<<LED_PIN);
    }

    // light LED_PIN after reset
    if( reset) {
      PORTA &= ~(1<<LED_PIN);
    }

    if( fiftieths == 50) {
      // turn led on
      PORTA &= ~(1<<LED_PIN);
      fiftieths = 0;
      ++seconds;
      // rotator death protection
      if( seconds == DEADTIME && rotator_status) {
        rotator_status = 0;
        STOP_AZIMUT;
        STOP_ELEVATION;
      }
      if( seconds == 255) { seconds = DEADTIME;}
      reset = 0;
    }
  }
}

void put_char( char c) {
  // wait for sending last character
  while( uart_status & (1<<txbusy));

  // prepare new char for sending
  uart_txd = c;

  // tell the system to send
  uart_status |= (1<<txbusy);
}

void put_int( uint16_t i) {
  char c;
  // conversion of output to ASCII
  c = '0';
  while ( i >= 100) {
    i -= 100;
    ++c;
  }
  put_char( c);
  c = '0';
  while ( i >= 10) {
    i -= 10;
    ++c;
  }
  put_char( c);
  c = '0'+i;
  put_char( c);
}

void init( void) {

  // define timer0 for uart
  ticks = 0;
  fiftieths = 0;

  DDRB = 1<<LEFT_PIN | 1<<RIGHT_PIN | 1<<SPEEDCTRL_A | 1<<SPEEDCTRL_B;	  // define output pins
  DDRA = 1<<DOWN_PIN | 1<<UP_PIN | 1<<TX_PIN | 1<<LED_PIN;                    // define output pins
  PORTA |= 1<<TX_PIN | 1<<LED_PIN;	                            	        // write 1 to output
  PORTB |= 1<<SPEEDCTRL_A | 1<<SPEEDCTRL_B;               		        // write 1 to output

  TCCR0B = 0x01;               // set prescaler for timer0 to 1
  TCNT0L = TIMER0PRESET;				// preset for timer

  TIMSK = (1<<TOIE0); 		    // allow interrupts on timer0 overflow

//  TCCR1A = 0x21;              // enable PWM mode for OC1B - PB3, clear PB3 on compare match. Set when TCNT=0x01
//  TCCR1B = 0x01;              // prescaler = 1
//  OCR1C = 0x0a;               // the PWM counter is counting from 0 -

  next_write = 0;       // set up the buffer pointers

  uart_status = 0;			// nothing happening either tx or rx

  rotator_status = 0;     // the rotator is not moving

  // Configure ADC
  ADCSRA = 0b00000110;         // Prescaler = 64 >> clk=125kHz,  Single conversion

  ADMUX = ADC_AZIMUTH;
  ADCSRB = 0;                  // use Vcc as referenc and select input

  ADCSRA |= (1<<ADEN);         // enable the ADC

  ADCSRA |= (1<<ADSC);         // start AD conversion

  seconds = 0;                // just count seconds
  reset = 1;                  // show recovering from reset
  sei();                      // o.k. lets start the lion

  return;
}

int main(void) {
  const char* addr;
  char c;
  MCUSR = 0;
  wdt_disable();

  init();
  while( fiftieths < 10) {};  // wait for 1/5 second

  wdt_enable( WDTO_1S);     // the watchdog is running

  job = job_getversion;
  while(1) {
    if( job & job_getversion) {
      job &= ~job_getversion;
      addr = version;
      while(( c = pgm_read_byte( addr++)) != '\0' ) {
        put_char( c);
      }
    }
    if( job & job_get_help) {
      job &= ~job_get_help;
      addr = help;
      while(( c = pgm_read_byte( addr++)) != '\0' ) {
        put_char( c);
      }
    }
    if( job & job_get_reading || job & job_tuning) {
      put_char( '+');
      put_char( '0');
      put_int( bearing_a);
      put_char( '+');
      put_char( '0');
      put_int( bearing_b);
      put_char( '\n');
      if( job & job_get_reading) {
        job &= ~job_get_reading;
      }
    }
    if( job & job_get_azimut) {
      put_char( '+');
      put_char( '0');
      put_int( bearing_a);
      put_char( '\n');
      if( job & job_get_azimut) {
        job &= ~job_get_azimut;
      }
    }
    if( job & job_get_elevation) {
      put_char( '+');
      put_char( '0');
      put_int( bearing_b);
      put_char( '\n');
      if( job & job_get_elevation) {
        job &= ~job_get_elevation;
      }
    }
    wdt_reset();
  }
  return 0;
}
