#include <avr/sleep.h>
#include <avr/wdt.h>
#include <TinyGPS.h>
#include <stdio.h>
#include <util/crc16.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

volatile boolean f_wdt=1;

TinyGPS gps;

int hour = 0 , minute = 0 , second = 0, numbersats = 99, n, j, q, navmode, count = 0;
unsigned long date, time, chars, age;
char latbuf[12] = "0", lonbuf[12] = "0", superbuffer [120], altbuf [12] = "0", checksum [10];
long int ialt = 123;
float flat, flon;

struct t_mtab { char c, pat; } ;

struct t_mtab morsetab[] = {
  	{'.', 106},
	{',', 115},
	{'?', 76},
	{'/', 41},
        {'-', 97},
	{'A', 6},
	{'B', 17},
	{'C', 21},
	{'D', 9},
	{'E', 2},
	{'F', 20},
	{'G', 11},
	{'H', 16},
	{'I', 4},
	{'J', 30},
	{'K', 13},
	{'L', 18},
	{'M', 7},
	{'N', 5},
	{'O', 15},
	{'P', 22},
	{'Q', 27},
	{'R', 10},
	{'S', 8},
	{'T', 3},
	{'U', 12},
	{'V', 24},
	{'W', 14},
	{'X', 25},
	{'Y', 29},
	{'Z', 19},
	{'1', 62},
	{'2', 60},
	{'3', 56},
	{'4', 48},
	{'5', 32},
	{'6', 33},
	{'7', 35},
	{'8', 39},
	{'9', 47},
	{'0', 63}
} ;
#define N_MORSE  (sizeof(morsetab)/sizeof(morsetab[0]))

#define SPEED  (15)
#define DOTLEN  (1200/SPEED)
#define DASHLEN  (3*(1200/SPEED))

void dash()
{
  digitalWrite(8, HIGH) ;
  digitalWrite(4, HIGH) ;
  delay(DASHLEN);
  digitalWrite(8, LOW) ;
  digitalWrite(4, LOW) ;
  delay(DOTLEN) ;
}

void dit()
{
  digitalWrite(8, HIGH) ;
  digitalWrite(4, HIGH) ;
  delay(DOTLEN);
  digitalWrite(8, LOW) ;
  digitalWrite(4, LOW) ;
  delay(DOTLEN);
}

void
send(char c)
{
  int i ;
  if (c == ' ') {
    Serial.print(c) ;
    delay(7*DOTLEN) ;
    return ;
  }
  for (i=0; i<N_MORSE; i++) {
    if (morsetab[i].c == c) {
      unsigned char p = morsetab[i].pat ;
      Serial.print(morsetab[i].c) ;

      while (p != 1) {
          if (p & 1)
            dash() ;
          else
            dit() ;
          p = p / 2 ;
      }
      delay(2*DOTLEN) ;
      return ;
    }
  }
  /* if we drop off the end, then we send a space */
  Serial.print("?") ;
}

void
sendmsg(char *str)
{
  while (*str)
    send(*str++) ;
  Serial.println("");
}

void setup(){
  
  Serial.begin(9600);
  Serial.print("Starting...");
  pinMode(4, OUTPUT); //Radio Tx0
  pinMode(5, OUTPUT); //Radio Tx1
  pinMode(6, OUTPUT); //Radio En
  pinMode(8, OUTPUT); //LED
  
  digitalWrite(6, LOW);
  
  setupGPS();
  
  // CPU Sleep Modes 
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)

  cbi( SMCR,SE );      // sleep enable, power down mode
  cbi( SMCR,SM0 );     // power down mode
  sbi( SMCR,SM1 );     // power down mode
  cbi( SMCR,SM2 );     // power down mode

  setup_watchdog(9);
}

//****************************************************************
void loop(){

  if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    f_wdt=0;       // reset flag
   
   count++;
   
   if((count % 100) == 0) {
     navmode = 0;
     setupGPS();
   }
   
   
   if((count % 2) == 0) {
   Serial.println("$PUBX,00*33"); //Poll GPS
      wdt_reset();
      while (Serial.available())
      {
        int c = Serial.read();
        if (gps.encode(c))
        {
          digitalWrite(8, HIGH);
          //Get Data from GPS library
          //Get Time and split it
          gps.get_datetime(&date, &time, &age);
          hour = (time / 1000000);
          minute = ((time - (hour * 1000000)) / 10000);
          second = ((time - ((hour * 1000000) + (minute * 10000))));
          second = second / 100;
          wdt_reset();
          numbersats = gps.sats();
          
          //Get Position
          gps.f_get_position(&flat, &flon);
      
          //convert float to string
          dtostrf(flat, 7, 4, latbuf);
          dtostrf(flon, 7, 4, lonbuf);
          
          wdt_reset();
          //just check that we are putting a space at the front of lonbuf
          if(lonbuf[0] == ' ')
          {
            lonbuf[0] = '+';
          }
          
          // +/- altitude in meters
          ialt = (gps.altitude() / 100);   
          digitalWrite(8, LOW);
        }
      }
    
    wdt_reset();
    
    n=sprintf (altbuf, "%05ld", ialt);
    n=sprintf (superbuffer, "$$PICO,%d,%02d:%02d:%02d,%s,%s,%ld,%d,%d", count, hour, minute, second, latbuf, lonbuf, ialt, numbersats, navmode);
   
    n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));
      
    
    digitalWrite(6, HIGH); // Enable Radio
    delay(2000);
    wdt_reset();
    rtty_txstring(altbuf);
    rtty_txstring(superbuffer);
    delay(100);
    
    digitalWrite(6, LOW);
  }
  else {
    digitalWrite(6, HIGH); // Enable Radio
    delay(2000);
    sendmsg("PICO,");
    sendmsg(altbuf);
    delay(100);
    digitalWrite(6, LOW);
  }
  system_sleep();
  
  }
}

// ------------------------
// RTTY Functions - from RJHARRISON's AVR Code
void rtty_txstring (char * string)
{
	/* Simple function to sent a char at a time to 
	** rtty_txbyte function. 
	** NB Each char is one byte (8 Bits)
	*/
	char c;
	c = *string++;
	while ( c != '\0')
	{
		rtty_txbyte (c);
		c = *string++;
	}
}

void rtty_txbyte (char c)
{
	/* Simple function to sent each bit of a char to 
	** rtty_txbit function. 
	** NB The bits are sent Least Significant Bit first
	**
	** All chars should be preceded with a 0 and 
	** proceded with a 1. 0 = Start bit; 1 = Stop bit
	**
	** ASCII_BIT = 7 or 8 for ASCII-7 / ASCII-8
	*/
	int i;
	rtty_txbit (0); // Start bit
	// Send bits for for char LSB first	
	for (i=0;i<8;i++)
	{
		if (c & 1) rtty_txbit(1); 
			else rtty_txbit(0);	
		c = c >> 1;
	}
	rtty_txbit (1); // Stop bit
        rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
		if (bit)
		{
		  // high
                    
                    digitalWrite(5, HIGH);  
                    digitalWrite(4, LOW);
		}
		else
		{
		  // low
                    
                    digitalWrite(4, HIGH);
                    digitalWrite(5, LOW);
		}
		//delayMicroseconds(20500); // 10000 = 100 BAUD 20150
                delayMicroseconds(20000); // 10000 = 100 BAUD 20150
}

void setupGPS() {
    //Turning off all GPS NMEA strings apart on the uBlox module
  Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
  delay(1000);
  Serial.println("$PUBX,40,GGA,0,0,0,0*5A");
  delay(1000);
  Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
  delay(1000);
   wdt_reset();
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  delay(1000);
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  delay(1000);
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
  wdt_reset();
  delay(3000); // Wait for the GPS to process all the previous commands
  
}


uint16_t gps_CRC16_checksum (char *string)
{
	size_t i;
	uint16_t crc;
	uint8_t c;
 
	crc = 0xFFFF;
 
	// Calculate checksum ignoring the first two $s
	for (i = 2; i < strlen(string); i++)
	{
		c = string[i];
		crc = _crc_xmodem_update (crc, c);
	}
 
	return crc;
}


//****************************************************************  
// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep() {

  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here

    sleep_disable();                     // System continues execution here when watchdog timed out 
    sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON

}

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  Serial.println(ww);


  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);


}

//****************************************************************  
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}
