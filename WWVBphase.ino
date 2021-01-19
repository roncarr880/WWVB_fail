// A minimum design for a WWVB receiver/clock decoding the phase encoded time signal.
// Target Arduino is a Nano.  A SA/NE602/612 is the receiver, and an I2C OLED display is used.
// Nano generates the vfo clock, samples audio, and processes the signal phase changes.


#include <Arduino.h>
#include <avr/interrupt.h>
#include <OLED1306_Basic.h>
#include <TimerOne.h>

#define ROW0 0          // text based rows for the 128x64 OLED
#define ROW1 8
#define ROW2  16
#define ROW3  24
#define ROW4  32
#define ROW5  40
#define ROW6  48
#define ROW7  56


const int vfoPin = 9;

OLED1306 LCD;            // a modified version of LCD_BASIC by Rinky-Dink Electronics

extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
extern unsigned char BigNumbers[];

//  I2C buffers and indexes
#define I2TBUFSIZE 128             // size power of 2. Size 128 can buffer a full row
#define I2RBUFSIZE 16              // set to expected #of reads in row, power of 2
#define I2INT_ENABLED 0            // 0 for polling in loop or timer, 1 for TWI interrupts
unsigned int i2buf[I2TBUFSIZE];   // writes
uint8_t i2rbuf[I2RBUFSIZE];       // reads
volatile uint8_t i2in,i2out;
volatile uint8_t i2rin,i2rout;
//uint8_t gi2state;                  // global copy of i2c state


void setup(void){
  
  Serial.begin(38400);                  // for debug

  // pinMode(vfoPin,OUTPUT);               // works with or without this 
  // generate the vfo signal on pin 9.  WWVB signal is 60khz.
  Timer1.initialize(17);                // 16 -->62500 hz.  17 -->58823 hz.  Audio IF is 2500 or 1177
                                        // scope says 62509.3     58832.3  or about 9.3 fast
  Timer1.pwm(vfoPin, 512);              // 50/50 clock 

  // generate an interrupt for the I2C processing, timer 0 compare.
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  LCD.InitLCD();                        // using a modified Nokia library for the OLED
  LCD.setFont(SmallFont);
  LCD.clrScr();
  LCD.print("WWVB clock test",0,ROW0);

  analogRead(A7);                      // let the arduino core setup the analog read registers
  delay(3000);


  // !!! testing
  LCD.setFont(MediumNumbers);
}

// timer polling.  about 1000 cps or 8k baud screen writes
ISR(TIMER0_COMPA_vect){
   i2poll();
}

// TWI interrupt version
// if twi interrupts enabled, then need a handler
// Stop does not produce an interrupt, so need to also poll in loop or via the timer 
//ISR(TWI_vect){
//  i2poll();
//  if( gi2state == 0 ) i2poll();   // needed to get out of state zero, else double ints
// // ++ints;
//}


void loop(void){
int test;
//static int high;
//static int low;
static int32_t count;
static int blipc;
int blip;
static float ave;
static int sig;
static float total;
static int samples;
static int glitch;

   test = analogRead(A7);  
  // Serial.print(test); Serial.write(' ');  
   test -= 430;                     // 430   779
   //test = abs(test);
   if( test < 0 ) test = -test;
   total+= test;
   ++samples;
   
 if( millis() - count >= 50 ){       // sample 20 times a second
   count = millis();
   if( ++blipc == 20 ){
      blip = -1;
      blipc = 0;
   }
   else blip = 0;
   total = total/samples;
   ave = 99*ave + total;         // was 31/32
   ave/=100;   
   LCD.printNumI(total,20,ROW3,3,'/');  // LCD.clrRow(ROW3,80);
   LCD.printNumI(ave, 20,ROW6,3,'/');
   if( total > ave-1){
       if( glitch == 1 ) sig = 20;
       else ++glitch;
   }
   if( total < ave-1 ){
    if( glitch == 0 ) sig = 0;
    else --glitch;
   } 
   Serial.print(blip); Serial.write(' ');
   Serial.print(sig); Serial.write(' ');
   Serial.print(ave-1); Serial.write(' ');
   Serial.println(total);

   total = samples = 0;
 }
}



/*****  Non-blocking  I2C  functions   ******/

void i2init(){
  TWBR = 12;   //8 500k,  12 400k, 72 100k   for 16 meg clock. ((F_CPU/freq)-16)/2
  TWDR = 0xFF;
  PRR &= 0x7F;
  TWSR = 1<<TWEN;
}
// use some upper bits in the buffer for control
#define ISTART 0x100
#define ISTOP  0x200

void i2start( unsigned char adr ){
unsigned int dat;
  // shift the address over and add the start flag
  dat = ( adr << 1 ) | ISTART;
  i2send( dat );
}


void i2send( unsigned int data ){   // just save stuff in the buffer
uint8_t  next;

  // check for buffer full
  next = (i2in + 1) & (I2TBUFSIZE-1);
  if( next == i2out ){
     while( next == i2out ){
         noInterrupts();
         i2poll();        // wait for i2out to move
         interrupts();
     }
    //++waits;
  }
  
  i2buf[i2in++] = data;
  i2in &= (I2TBUFSIZE - 1);
  
  noInterrupts();
  i2poll();         // wake up interrupts when new data arrives in case stopped in state 0 or 3
  interrupts();     // using polling, but can keep this here

}

void i2stop( ){
   i2send( ISTOP );   // que a stop condition
}


void i2flush(){  // call flush to empty out the buffer, waits on I2C transactions completed 
uint8_t  ex;

  ex = 1;
  while(ex){
     noInterrupts();
     ex = i2poll(); 
     interrupts();
  }
}

// queue a read that will complete later
void i2queue_read( unsigned char adr, unsigned char reg, unsigned char qty ){
unsigned int dat;

   i2start( adr );
   i2send( reg );
   // i2stop();     // or repeated start
   dat = ((unsigned int)qty << 10) | ( adr << 1 ) | ISTART | 1;   // a start with the read bit set
   i2send( dat );
   i2stop();        // stop to complete the transaction
}

int i2read_int(){     // returns 2 values in i2c read queue as a signed integer
int data;

      if( i2rout == i2rin ) return 0;
      data = i2rbuf[i2rout++];
      i2rout &= (I2RBUFSIZE-1);
      data <<= 8;
      if( i2rout == i2rin ) return data;
      data |= i2rbuf[i2rout++];
      i2rout &= (I2RBUFSIZE-1);
      return data;
}

uint8_t i2available(){
uint8_t qty;

     qty = i2rin - i2rout;
     if( qty > I2RBUFSIZE ) qty += I2RBUFSIZE;    // some funky unsigned math
     return qty;
}



uint8_t i2poll(){    // everything happens here.  Call this from loop. or interrupt
static  uint8_t state = 0;
static unsigned int data;
//static uint8_t delay_counter;
static unsigned int read_qty;
static uint8_t first_read;
   
   switch( state ){
     
      case 0:      // idle state or between characters
        if( i2in != i2out ){   // get next character
           data = i2buf[i2out++];
           i2out &= (I2TBUFSIZE - 1 );
         
           if( data & ISTART ){   // start
              if( data & 1 ) read_qty = data >> 10;     // read queued
              data &= 0xff;
              // set start condition
              TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (I2INT_ENABLED); 
              state = 1; 
           }
           else if( data & ISTOP ){  // stop
              // set stop condition
              TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) | (I2INT_ENABLED);
              state = 3;
           }
           else{   // just data to send
              TWDR = data;
              TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);
              //delay_counter = 5;   // delay for transmit active to come true
              state = 2;
           }
        }
        else TWCR = (1<<TWEN);      // stop interrupts, keep enabled
      break; 
      case 1:  // wait for start to clear, send saved data which has the device address
         if( (TWCR & (1<<TWINT)) ){
            state = ( data & 1 ) ? 4 : 2;    // read or write pending?
            first_read = 1;
            TWDR = data;
            TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);
            //delay_counter = 5;
         }
      break;
      case 2:  // wait for done
         if( (TWCR & (1<<TWINT)) ){  
            state = 0;
         }
      break;
      case 3:  // wait for stop to clear. TWINT does not return to high and no interrupts happen on stop.
         if( (TWCR & (1<<TWSTO)) == 0 ){
            state = 0;
         } 
      break;
      case 4:  // read until count has expired
         if( (TWCR & (1<<TWINT)) ){
            // read data
            if( first_read ) first_read = 0;       // discard the 1st read, need 8 more clocks for real data
            else{
               i2rbuf[i2rin++] = TWDR;
               i2rin &= ( I2RBUFSIZE - 1 );
               if( --read_qty == 0 ) state = 0;    // done
            }
            
            if( read_qty ){                        // any left ?
               if( read_qty > 1 ) TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (I2INT_ENABLED);  // not the last read
               else TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);                            // nack the last read
            }
            //delay_counter = 5;
         }
      break;    
   }
   
   //gi2state = state;
   if( i2in != i2out ) return (state + 8);
   else return state;
}
