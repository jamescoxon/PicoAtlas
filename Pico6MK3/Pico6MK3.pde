#include <Wire.h>
#include <BMP085.h>
#include <SPI.h>
#include <RFM22.h>
#include "TinyGPS.h"

//Setup radio on SPI with NSEL on pin 10
rfm22 radio1(10);

//Initiate GPS
TinyGPS gps;

//
BMP085 bmp;

//Variables
long int ialt = 123, lat = 0, lon = 0, pressure = 0;
int hour = 0 , minute = 0 , second = 0, count = 0, switchMode = 1, n, q, j, radioOn = 0;
unsigned long date, time, chars, age;
char superbuffer [80];

int temperature = 0;
//Functions

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
                  radio1.setFrequency(434.2015);
		}
		else
		{
		  // low
                  radio1.setFrequency(434.201);
		}
		//delayMicroseconds(20500); // 10000 = 100 BAUD 20150
                //delayMicroseconds(20000); // 10000 = 100 BAUD 20150
                delayMicroseconds(19500); // 10000 = 100 BAUD 20150

}

//***************Hellschreiber******************

struct t_htab { char c; int hellpat[5]; } ;

struct t_htab helltab[] = {

  {'1', { B00000100, B00000100, B01111100, B00000000, B00000000 } },
  {'2', { B01001000, B01100100, B01010100, B01001100, B01000000 } },
  {'3', { B01000100, B01000100, B01010100, B01010100, B00111100 } },
  {'4', { B00011100, B00010000, B00010000, B01111100, B00010000 } },
  {'5', { B01000000, B01011100, B01010100, B01010100, B00110100 } },
  {'6', { B00111100, B01010010, B01001010, B01001000, B00110000 } },
  {'7', { B01000100, B00100100, B00010100, B00001100, B00000100 } },
  {'8', { B00111100, B01001010, B01001010, B01001010, B00111100 } },
  {'9', { B00001100, B01001010, B01001010, B00101010, B00111100 } },
  {'0', { B00111000, B01100100, B01010100, B01001100, B00111000 } },
  {'A', { B01111000, B00101100, B00100100, B00101100, B01111000 } },
  {'B', { B01000100, B01111100, B01010100, B01010100, B00101000 } },
  {'C', { B00111000, B01101100, B01000100, B01000100, B00101000 } },
  {'D', { B01000100, B01111100, B01000100, B01000100, B00111000 } },
  {'E', { B01111100, B01010100, B01010100, B01000100, B01000100 } },
  {'F', { B01111100, B00010100, B00010100, B00000100, B00000100 } },
  {'G', { B00111000, B01101100, B01000100, B01010100, B00110100 } },
  {'H', { B01111100, B00010000, B00010000, B00010000, B01111100 } },
  {'I', { B00000000, B01000100, B01111100, B01000100, B00000000 } },
  {'J', { B01100000, B01000000, B01000000, B01000000, B01111100 } },
  {'K', { B01111100, B00010000, B00111000, B00101000, B01000100 } },
  {'L', { B01111100, B01000000, B01000000, B01000000, B01000000 } },
  {'M', { B01111100, B00001000, B00010000, B00001000, B01111100 } },
  {'N', { B01111100, B00000100, B00001000, B00010000, B01111100 } },
  {'O', { B00111000, B01000100, B01000100, B01000100, B00111000 } },
  {'P', { B01000100, B01111100, B01010100, B00010100, B00011000 } },
  {'Q', { B00111000, B01000100, B01100100, B11000100, B10111000 } },
  {'R', { B01111100, B00010100, B00010100, B00110100, B01011000 } },
  {'S', { B01011000, B01010100, B01010100, B01010100, B00100100 } },
  {'T', { B00000100, B00000100, B01111100, B00000100, B00000100 } },
  {'U', { B01111100, B01000000, B01000000, B01000000, B01111100 } },
  {'V', { B01111100, B00100000, B00010000, B00001000, B00000100 } },
  {'W', { B01111100, B01100000, B01111100, B01000000, B01111100 } },
  {'X', { B01000100, B00101000, B00010000, B00101000, B01000100 } },
  {'Y', { B00000100, B00001000, B01110000, B00001000, B00000100 } },
  {'Z', { B01000100, B01100100, B01010100, B01001100, B01100100 } },
  {'.', { B01000000, B01000000, B00000000, B00000000, B00000000 } },
  {',', { B10000000, B10100000, B01100000, B00000000, B00000000 } },
  {'/', { B01000000, B00100000, B00010000, B00001000, B00000100 } },
  {'*', { B00000000, B00000000, B00000100, B00001110, B00000100 } }

};

#define N_HELL  (sizeof(helltab)/sizeof(helltab[0]))

void helldelay()
{
  //Slow
  delay(64);
  delayMicroseconds(900);

}

void on()
{
  radio1.write(0x07, 0x08); //on
  //radio1.setFrequency(434.201);
  helldelay();
  //radio1.setFrequency(434.301);
  radio1.write(0x07, 0x01); //off
}

void hellsend(char c)
{
  int i ;
  if (c == ' ') {
      for (int d=0; d<14; d++){
        helldelay();  
      }
    return ;
  }
  for (i=0; i<N_HELL; i++) {
    if (helltab[i].c == c) {
      //Serial.print(helltab[i].c) ;
      
      for (j=0; j<=4; j++) 
      {
        byte mask = B10000000;
        for (q=0; q<=6; q++)
        {      
          if(helltab[i].hellpat[j] & mask) {
            on();
          } else {
            helldelay();
          }
          mask >>= 1;
        }
      }
      for (int d=0; d<14; d++){
        helldelay();  
      }
      return ;
    }
  }
  /* if we drop off the end, then we send a space */
  //Serial.print("?") ;
}

void hellsendmsg(char *str)
{
  radio1.setFrequency(434.201);
  delay(1000);
  while (*str)
    hellsend(*str++) ;
  //Serial.println("");
}

//**************GPS Setup*******************

void setupGPS() {
  //Turning off all GPS NMEA strings apart on the uBlox module
  Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
  delay(1000);
  Serial.println("$PUBX,40,GGA,0,0,0,0*5A");
  delay(1000);
  Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
  delay(1000);
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  delay(1000);
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  delay(1000);
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
  delay(3000); // Wait for the GPS to process all the previous commands
  
  //set GPS to Cyclic mode
  uint8_t CFG_PM2[] = {0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x90,
  0x02, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x87, 0x02, 0x00, 0x00,
  0xFF, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0x97, 0xD9};
    
  sendUBX(CFG_PM2, sizeof(CFG_PM2)/sizeof(uint8_t));
  
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.print(MSG[i], BYTE);
  }
  //Serial.println();
}

//************Other Functions*****************

unsigned int gps_checksum (char * string)
{	
	unsigned int i;
	unsigned int XOR;
	unsigned int c;
	// Calculate checksum ignoring any $'s in the string
	for (XOR = 0, i = 0; i < strlen(string); i++)
	{
		c = (unsigned char)string[i];
		if (c != '$') XOR ^= c;
	}
	return XOR;
}

void setupRadio(){
  
  digitalWrite(5, LOW);
  
  delay(1000);
  
  rfm22::initSPI();

  radio1.init();
  
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
  
  radio1.write(0x71, 0x00); // unmodulated carrier
  
  radio1.setFrequency(434.201);
  
  radio1.write(0x07, 0x08); // turn tx on
  delay(1000);
  radio1.write(0x07, 0x01); // turn tx off
  
}

//****************Main Program****************
void setup() {
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  Serial.begin(9600);
  
  bmp.begin(); 
  
  delay(1000);
  
  setupGPS();
  

}

void loop() {
  count++;
  Serial.println("$PUBX,00*33"); //Poll GPS
  while (Serial.available())
  {
    digitalWrite(13, HIGH);
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
      
      //Get Position
      gps.get_position(&lat, &lon, &age);
      
      // +/- altitude in meters
      ialt = (gps.altitude() / 100); 
    }
    digitalWrite(13, LOW);
  }
  
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();

  n=sprintf (superbuffer, "PICO,%d,%02d:%02d:%02d,%ld,%ld,%ld,%d,%ld", count, hour, minute, second, lat, lon, ialt, temperature, pressure);
  n = sprintf (superbuffer, "%s*%02X\n", superbuffer, gps_checksum(superbuffer));
  
  if(lat != 0){
    if(radioOn == 0){
      setupRadio();
      radioOn = 1;
    }
    
    if((count % 10) == 0) {
      if (switchMode == 0) {
        switchMode = 1;
      }
      else{
        switchMode = 0;
      }
    }
    
    if (switchMode == 0) {
       hellsendmsg(superbuffer);
       radio1.write(0x07, 0x01); // turn tx off
    }
    else{
      radio1.write(0x07, 0x08); // turn tx on
      delay(5000);
      rtty_txstring("$$$$");
      rtty_txstring(superbuffer);
      radio1.write(0x07, 0x01); // turn tx off
      
    }
  }
  else{
    Serial.println(superbuffer);
    delay(5000);
  }
  
}
