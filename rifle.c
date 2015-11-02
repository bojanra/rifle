/*****************************************************************************
  Rifle - antenna rotator interface for YAESU G-1000DXC series rotator

  -----------------------------------------------------------------------------
  2012/7/11 - 2013/10/28 - 20141/24

Basically all work is build around the timer wich is 4 times the baudrate and 
implements a software uart. The target is ATTINY 26.

In normal mode the Led is blinking Duty Cyle 10%.
When incrementing speed for rotator A the DC is 10-10%;

We can control 2 yaesu rotators A and B. 

When more than (60 seconds) DEADTIME after each start of movement the target is not reached 
we stop the rotator movement.

The watchdog is active via the main loop.

Accept these commands:
g - get readings:
    bearing device A,
    bearing device B,
    fieldstrength voltage (mosley),
    rotator_status
    (g150.234.001.0\n) 3 digits rotator A,3 digits rotator B,3 digits voltage,1 digit rotator_status

    Legend for rotator_status:
    0 - stopped
    1 - b moving
    2 - a moving
    3 - a and b moving

t - tuning mode on - continously send readings
T - stop tuning mode
r - reset via watchdog

axxx\r - set bearing device A
bxxx\r - set bearing device B

v - Version string
  - stop motion & tuning mode & debug output (space)

Additional feature antena death protection. Stop rotator movement after 60 seconds.

 *****************************************************************************/
// internal clock 
#define F_CPU 8000000
#define TIMER0PRESET (255-98)   //  255-t_bit  ,  t_bit = 1/2400/4*F_CPU/prescaler 
#define DEADTIME 60             // after DEADTIME seconds since start of movement the rotator is stopped

#define DEBUG 1

#include <avr/io.h>
// #include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

const char version[] PROGMEM = "Rifle V4.5\n";

#define	UARTTX      PA7
#define UARTRX      PA6
#define LED         PA3
#define MOSL        PA1   // input for received fieldstrength

#define LEFT_A      PB0   // rotator A
#define RIGHT_A     PB1    
#define SPEEDCTRL_A PB3
#define SPEED_A     OCR1B
#define BEAR_A      PA0   // input for rotator A bearing


#define LEFT_B      PA4   // rotator B
#define RIGHT_B     PA5
#define SPEEDCTRL_B PB6
#define BEAR_B      PA2

#define ADC_MUX_BEAR_A  0   // read bearing from port PA0
#define ADC_MUX_MOSL    1   // analog input from port PA1
#define ADC_MUX_BEAR_B  2   // read bearing from port PA2

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
#define job_timeout       32

// status codes
#define moving_a    1
#define moving_b    2

// clock times
static volatile uint8_t ticks;			// 192 of these make 1/50th of a second
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

static unsigned char buffer[8]; // circular buffer for receiver

// status bits
#define txbusy 0				// set if a byte is in transmission
#define rxbusy 1				// set if a byte is being received
#define rxframe 2				// set if the stop byte isn't a one

#define ABSDIFF(a, b) ((a) < (b)? ((b) - (a)): ((a) - (b))) 


ISR( TIMER0_OVF0_vect) {
  // the interrupt signal

  // we arrive here four times per baud rate
  // preset the timer0
  TCNT0 = TIMER0PRESET;				// preset for timer

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
        PORTA &= ~(1<<UARTTX);		// clear the start bit output
        uart_txbit++;
      }
      else {
        if( uart_txbit != 9) {
          // deal with the data bits
          if( uart_txd & 1)		// low bit set?
            PORTA |= (1<<UARTTX);	// then set the data stream bit
          else
            PORTA &= ~(1<<UARTTX);  // or clear, as required
          uart_txbit++;				// increment the bit count

          // and shift the data right
          uart_txd /= 2;
        }
        else {
          // deal with the stop bit
          PORTA |= (1<<UARTTX);
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
    if( (PINA & (1<<UARTRX)) == 0) {
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
        if ((PINA & (1<<UARTRX)) == 0) {
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
          if ((PINA & (1<<UARTRX)) != 0) {
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
          // test the received character
          if( uart_rxd == ' ') {
            // stop all
            rotator_status = 0;
            job = 0;
            PORTB &= ~(1<<LEFT_A);
            PORTB &= ~(1<<RIGHT_A);
            PORTA &= ~(1<<LEFT_B);
            PORTA &= ~(1<<RIGHT_B);
            next_write = 0;
          }
          else if( uart_rxd == 'v') {
            job |= job_getversion;
            next_write = 0;
          }
          else if( uart_rxd == 'g') {
            job |= job_get_reading;
            next_write = 0;
          }
          else if( uart_rxd == 'T') {
            job &= ~job_tuning;
            next_write = 0;
          }
          else if( uart_rxd == 'r' ) {
            while( 1) {
            }
          }
          else if( uart_rxd == 't' ) {
            job |= job_tuning;
            next_write = 0;
          }
          else if( uart_rxd == 'a' || uart_rxd == 'b' ) {
            next_write = 0;
            buffer[next_write++] = uart_rxd;
          }
          else if( uart_rxd == 13 && next_write == 4) {
            if( buffer[1] >= '0' && buffer[1] <= '9' &&
                buffer[2] >= '0' && buffer[2] <= '9' &&
                buffer[3] >= '0' && buffer[3] <= '9' ) {
              uint16_t target;
              target = 100*(buffer[1] & 0x0f)+10*(buffer[2] & 0x0f)+(buffer[3] & 0x0f);
              if( buffer[0] == 'a') {
                target_a = target;
                if( target_a <= 450) {
                  // target is o.k. start moving //
                  SPEED_A = 0;
                  if( target_a > bearing_a) {
                    // right
                    PORTB &= ~(1<<LEFT_A);
                    PORTB |= (1<<RIGHT_A);
                  }
                  else if( target_a < bearing_a) {
                    // left
                    PORTB &= ~(1<<RIGHT_A);
                    PORTB |= (1<<LEFT_A);
                  }
                  seconds = 0;
                  rotator_status |= moving_a;
                }
                else {
                  // stop moving //
                  rotator_status &= ~moving_a;
                  PORTB &= ~(1<<RIGHT_A);
                  PORTB &= ~(1<<LEFT_A);
                }
              }
              else if( buffer[0] == 'b') {
                target_b = target;
                if( target_b <= 450) {
                  // target is o.k. start moving //
                  if( target_b > bearing_b) {
                    // right
                    PORTA &= ~(1<<LEFT_B);
                    PORTA |= (1<<RIGHT_B);
                  }
                  else if( target_b < bearing_b) {
                    // left
                    PORTA &= ~(1<<RIGHT_B);
                    PORTA |= (1<<LEFT_B);
                  }
                  seconds = 0;
                  rotator_status |= moving_b;
                }
                else {
                  // stop moving //
                  rotator_status &= ~moving_b;
                  PORTA &= ~(1<<RIGHT_B);
                  PORTA &= ~(1<<LEFT_B);
                }
              }
            }
            next_write = 0;
          }
          else {
            // store the data into the buffer
            buffer[next_write++] = uart_rxd;
            // buffer protection
            next_write &= 0x07;
          }
        }
      }
    }
  }

  // increment the clock
  ticks++;
  if( ticks == 192) {
    // run this code every 20ms
    ticks = 0;
    fiftieths++;

    // ADC stuff
    if( (ADCSR & (1<<ADSC)) == 0) {
      if( ADMUX == 0) {
        // read bearing a
        bearing_a = ADCW;
        bearing_a >>= 1;
        ADMUX = ADC_MUX_MOSL;
      }
      else if( ADMUX & ADC_MUX_MOSL) {
        // read mosley
        mosley = ADCW;
        mosley >>= 2;
        ADMUX = ADC_MUX_BEAR_B;
      }
      else {
        // read bearing b
        bearing_b = ADCW;
        bearing_b >>= 1;
        ADMUX = ADC_MUX_BEAR_A;
      }
      ADCSR |= (1<<ADSC);        // start AD conversion
    }

    // movement control - do not overshoot - only check if moving
    if( (fiftieths % 10 == 0) && rotator_status) {
      // we have arrived at the target bearing
      if( target_a == bearing_a || 
          ((PORTB & (1<<LEFT_A)) && target_a > bearing_a) || 
          ((PORTB & (1<<RIGHT_A)) && target_a < bearing_a)) {
        rotator_status &= ~moving_a;
        PORTB &= ~(1<<RIGHT_A);
        PORTB &= ~(1<<LEFT_A);
      }
      if( target_b == bearing_b || 
          ((PORTA & (1<<LEFT_B)) && target_b > bearing_b) || 
          ((PORTA & (1<<RIGHT_B)) && target_b < bearing_b)) {
        rotator_status &= ~moving_b;
        PORTA &= ~(1<<RIGHT_B);
        PORTA &= ~(1<<LEFT_B);
      }
    }

    if( fiftieths == 5 || fiftieths == 15 || fiftieths == 30) {
      // turn led off
      PORTA |= (1<<LED);
    }

    // when moving do additional blink
    if( fiftieths == 25 && rotator_status) {
      // turn led on
      PORTA &= ~(1<<LED);
    }

    // light LED after reset
    if( reset) {
      PORTA &= ~(1<<LED);
    }

    if( fiftieths == 50) {
      // turn led on
      PORTA &= ~(1<<LED);
      fiftieths = 0;
      ++seconds;
      // rotator death protection
      if( seconds == DEADTIME && rotator_status) {
        PORTB &= ~(1<<LEFT_A);
        PORTB &= ~(1<<RIGHT_A);
        PORTA &= ~(1<<LEFT_B);
        PORTA &= ~(1<<RIGHT_B);
        job ^= job_timeout;
        rotator_status = 0;
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

  DDRB = 1<<LEFT_A | 1<<RIGHT_A | 1<<SPEEDCTRL_A | 1<<SPEEDCTRL_B;	  // define output pins
  DDRA = 1<<LEFT_B | 1<<RIGHT_B | 1<<UARTTX | 1<<LED;                 // define output pins
  PORTA |= 1<<UARTTX | 1<<LED;	                            	        // write 1 to output
  PORTB |= 1<<SPEEDCTRL_A | 1<<SPEEDCTRL_B;               		        // write 1 to output

  TCCR0 = 0x02;               // set prescaler for timer0 to 8
  TCNT0 = TIMER0PRESET;				// preset for timer

  TIMSK = (1<<TOIE0); 		    // allow interrupts on timer0 overflow

//  TCCR1A = 0x21;              // enable PWM mode for OC1B - PB3, clear PB3 on compare match. Set when TCNT=0x01
//  TCCR1B = 0x01;              // prescaler = 1 
//  OCR1C = 0x0a;               // the PWM counter is counting from 0 - 
//  SPEED_A = 0;                // default is lowest rotating speed

  next_write = 0;       // set up the buffer pointers

  uart_status = 0;			// nothing happening either tx or rx

  rotator_status = 0;     // the rotator is not moving

  // Configure ADC
  ADCSR = 0b00000110;         // Prescaler = 64 >> clk=125kHz,  Single conversion

  ADMUX = ADC_MUX_BEAR_A;     // use Vcc as referenc and select input

  ADCSR |= (1<<ADEN);         // enable the ADC

  ADCSR |= (1<<ADSC);         // start AD conversion

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
  while( seconds < 1) {};  // wait for 1 second

  wdt_enable(WDTO_500MS);     // the watchdog is running

  job = job_getversion;
  while(1) {
    if( job & job_timeout) {
      job &= ~job_timeout;
      put_char( 't');
      put_char( '\n');      
    }
    if( job & job_getversion) {
      job &= ~job_getversion;
      addr = version;
      while(( c = pgm_read_byte( addr++)) != '\0' ) {
        put_char( c);
      }
    }
    if( job & job_get_reading || job & job_tuning) {
      put_char( 'g');
      put_int( bearing_a);
      put_char( '.');
      put_int( bearing_b);
      put_char( '.');
      put_int( mosley+1);
      put_char( '.');
      c = '0';
      c += rotator_status;
      put_char( c);
      put_char( '\n');
      if( job & job_get_reading) {
        job &= ~job_get_reading;
      }
    }
    wdt_reset();
  }
  return 0;
}

