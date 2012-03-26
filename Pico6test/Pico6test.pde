
#include <SPI.h>
#include <RFM22.h>

//Setup radio on SPI with NSEL on pin 10
rfm22 radio1(10);


//Variables
long int ialt = 123, lat = 0, lon = 0, pressure = 0;
int hour = 0 , minute = 0 , second = 0, count = 0, slowfeld = 1, n, q, j, radioOn = 0;
unsigned long date, time, chars, age;
char superbuffer [80];

//BMP085 variables
#define I2C_ADDRESS 0x77 //77?

const unsigned char oversampling_setting = 3; //oversamplig for measurement
const unsigned char pressure_waittime[4] = { 5, 8, 14, 26 };

//just taken from the BMP085 datasheet
int ac1, ac2, ac3, b1, b2, mb, mc, md, temperature = 0;
unsigned int ac4, ac5, ac6;

float freq = 434.2;

char freqbuf[12] = "0";

//Functions

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
                  radio1.setFrequency(434.201);
                  digitalWrite(13, HIGH);
		}
		else
		{
		  // low
                  radio1.setFrequency(434.2015);
                  digitalWrite(13, LOW);
		}
		//delayMicroseconds(20500); // 10000 = 100 BAUD 20150
                //delayMicroseconds(20000); // 10000 = 100 BAUD 20150
                delayMicroseconds(19500); // 10000 = 100 BAUD 20150

}

void setupRadio(){
  
  digitalWrite(5, LOW);
  
  delay(1000);
  
  rfm22::initSPI();

  radio1.init();
  
  radio1.write(0x71, 0x00); // unmodulated carrier
  
  radio1.setFrequency(434.200);
  
  radio1.write(0x07, 0x08); // turn tx on
  delay(1000);
  radio1.write(0x07, 0x01); // turn tx off
  
}

//****************Main Program****************
void setup() {
  pinMode(5, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(5, HIGH);
  Serial.begin(9600);
  
  delay(1000);
  
  setupRadio();
  
  radio1.write(0x07, 0x08); // turn tx on
  

}

void loop() {
    rtty_txstring("TEST TEST TEST TEST");
    delay(5000);
    Serial.println("Done");

  
}
