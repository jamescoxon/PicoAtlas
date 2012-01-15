
//****************************************************************
/*
 * Watchdog Sleep Example 
 * Demonstrate the Watchdog and Sleep Functions
 * 
 
 * KHM 2008 / Lab3/  Martin Nawrath nawrath@khm.de
 * Kunsthochschule fuer Medien Koeln
 * Academy of Media Arts Cologne
 
 */
//****************************************************************

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

byte del;
int cnt, navmode;
int count = 0, gpsonoff = 0;
char buf5[10];

int hour = 0 , minute = 0 , second = 0, oldsecond = 0;
unsigned long date, time, chars, age;
char latbuf[12] = "0", lonbuf[12] = "0";
long int ialt = 123;
int numbersats = 99, battV = 0;
float flat, flon;
int loopnumber = 5, gpsnumber = 75, gpsrepeat = 0;;

void setup(){

  pinMode(2, OUTPUT); //GPS Pwr
  offGPS(); // turn the GPS off so save power
  
  Serial.begin(9600);
  pinMode(7, OUTPUT); //Radio Tx0
  pinMode(8, OUTPUT); //Radio Tx1
  pinMode(9, OUTPUT); //Radio En
  
  digitalWrite(9, LOW);
  
  
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

    char superbuffer [120];
    char checksum [10];
    int n;

  if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    f_wdt=0;       // reset flag
    
    count++;
    battV = analogRead(1);
    
    if (battV < 745) {
      offGPS();
    }
     
    //Check the battery voltage - if it is too low then switch to transmitting with longer intervals to conserve power
    if (battV < 725) {
      loopnumber = 20;
    }
    else {
      loopnumber = 5;
    }
    
    //Check the battery voltage - if it is high then check the GPS more often   
    if (battV > 800) {
      gpsnumber = 40;
    }
    else {
      gpsnumber = 75;
    }
    
    if((count % gpsnumber) == 0) {
      if (gpsonoff == 1) {
        offGPS();
      }
      else {
        if (battV < 735) {
          offGPS();
        }
        else {
          onGPS();
          wdt_reset();
          delay(3000);
          wdt_reset();
          setupGPS();
          wdt_reset();
          delay(5000);   //ADDED IN DELAY HERE TO SEE IF THAT ALLOWS THE GPS TO STOP PROVIDING OLD DATA 
          wdt_reset();
          gpsonoff = 1;
        }
      }
    }
    
    if((count % loopnumber) == 0) {
      
      if(numbersats >= 3 && numbersats != 99) {
        offGPS();
      }
      
      if (gpsonoff == 1){
        Serial.println("$PUBX,00*33"); //Poll GPS
        wdt_reset();
        while (Serial.available())
        {
          int c = Serial.read();
          if (gps.encode(c))
          {
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
          }
        }
      }
      wdt_reset();
      
      digitalWrite(9, HIGH); // Enable Radio
      delay(2000);
      wdt_reset();
      if (gpsonoff == 1){
        n=sprintf (superbuffer, "$$PICO,%d,%02d:%02d:%02d,%s,%s,%ld,%d,%d;%d", count, hour, minute, second, latbuf, lonbuf, ialt, numbersats, navmode, battV);
      }
      else {
        n=sprintf (superbuffer, "$$PICO,%d,%d", count, battV);
      }
      
      n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));
      
      if(numbersats >= 3 && numbersats != 99 && gpsrepeat > 0) {
        offGPS();
        for (int i=0; i <= 4  ; i++){
          wdt_reset();
          rtty_txstring("UUUUU");
          rtty_txstring(superbuffer);
          delay(1000);
        }
      }
      else {
        wdt_reset();
        gpsrepeat++;
        rtty_txstring("UUUUU");
        rtty_txstring(superbuffer);
        delay(100);
      }
      
      digitalWrite(9, LOW); // Disable Radio
    }
    
    pinMode(9,INPUT); // set all used port to intput to save power
    pinMode(8,INPUT); // set all used port to intput to save power
    pinMode(7,INPUT); // set all used port to intput to save power

    system_sleep();

    pinMode(9,OUTPUT); // set all ports into state before sleep
    pinMode(8,OUTPUT); // set all ports into state before sleep
    pinMode(7,OUTPUT); // set all ports into state before sleep

  }
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
                    
                    digitalWrite(7, HIGH);  
                    digitalWrite(8, LOW);
		}
		else
		{
		  // low
                    
                    digitalWrite(8, HIGH);
                    digitalWrite(7, LOW);
		}
		//delayMicroseconds(20500); // 10000 = 100 BAUD 20150
                delayMicroseconds(20000); // 10000 = 100 BAUD 20150
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.print(MSG[i], BYTE);
  }
  Serial.println();
}

// Get the current NAV5 mode
int getUBXNAV5() {

	uint8_t b;
	uint8_t byteID = 0;
	int startTime = millis();

	// Poll/query message
	uint8_t getNAV5[] = { 0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84 };
	
	// First few bytes of the poll response
	uint8_t response[] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF};
	
	// Interrogate Mr GPS...
	sendUBX(getNAV5, sizeof(getNAV5)/sizeof(uint8_t));
	
	// Process his response...
	while (1) {
		
		// Timeout if no valid response in 3 seconds
		if (millis() - startTime > 3000) { 
			return -1;
		}

		// Make sure data is available to read
		if (Serial.available()) {
			b = Serial.read();

			// 8th byte is the nav mode
			if (byteID == 8) {
				return b;
			} 
			// Make sure the response matches the expected preamble
			else if (b == response[byteID]) { 
				byteID++;
			} else {
				byteID = 0;	// Reset and look again, invalid order
			}
			 
		}
	}

}

void onGPS(){        
  //turn on GPS
  digitalWrite(2, HIGH);
  numbersats = 99;
  flat = 0.0;
  flon = 0.0;
}

void offGPS(){
  //turn off GPS
  digitalWrite(2, LOW);
  gpsonoff = 0;
  numbersats = 99;
  flat = 0.0;
  flon = 0.0;
  gpsrepeat = 0;
}

void setupGPS() {
    //Turning off all GPS NMEA strings apart on the uBlox module
  Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
  Serial.println("$PUBX,40,GGA,0,0,0,0*5A");
  Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
  wdt_reset();
  delay(3000); // Wait for the GPS to process all the previous commands
  
 // Check and set the navigation mode (Airborne, 1G)   
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  navmode = getUBXNAV5();
 
  wdt_reset();
  delay(500);
  
  //set GPS to Eco mode (reduces current by 4mA)
  uint8_t setEco[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x00, 0x04, 0x1D, 0x85};
  sendUBX(setEco, sizeof(setEco)/sizeof(uint8_t));
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
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}
