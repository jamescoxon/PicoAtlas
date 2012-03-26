#include <TinyGPSUblox.h>
#include <SPI.h>
#include <RFM22.h>
#include <util/crc16.h>


//Setup radio on SPI with NSEL on pin 9
rfm22 radio1(10);

//Initiate GPS
TinyGPS gps;

//Variables
long int ialt = 123, lat = 0, lon = 0;
int hour = 0 , minute = 0 , second = 0, count = 0, n, radioOn = 0, navmode = 9, firstlock = 0;
unsigned long date, time, age;
char superbuffer [80];

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
                delayMicroseconds(19500); // 10000 = 100 BAUD 20150

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
  
}

void setupGPSpower() {
  //Set GPS to Power Save Mode
  uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 }; //Poll NAV5 status
  
  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
  //Serial.println();
}

//************Other Functions*****************

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

void setupRadio(){
  
  digitalWrite(9, LOW);
  
  delay(1000);
  
  rfm22::initSPI();

  radio1.init();
  
  radio1.write(0x71, 0x00); // unmodulated carrier
 
  
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
  
  radio1.setFrequency(434.201);
  
  radio1.write(0x07, 0x08); // turn tx on
  delay(1000);
  rtty_txstring("$$$$ATLAS");
  radio1.write(0x07, 0x01); // turn tx off
  
  
}

//****************Main Program****************
void setup() {
  pinMode(9, OUTPUT);
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);
  digitalWrite(9, HIGH);
  Serial.begin(9600);
  
  delay(5000);
  
  setupGPS();

  
  digitalWrite(A3, LOW);
}

void loop() {
  
  count++;
  Serial.println("$PUBX,00*33"); //Poll GPS
  while (Serial.available())
  {
    digitalWrite(A3, HIGH);
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
      
      if(firstlock == 0 && lat != 0){
          setupGPSpower();
          firstlock = 1;
      }
    }
    digitalWrite(A3, LOW);
  }
  
  n=sprintf (superbuffer, "PICO,%d,%02d:%02d:%02d,%ld,%ld,%ld,%d", count, hour, minute, second, lat, lon, ialt, navmode);
  n = sprintf (superbuffer, "%s*%02X\n", superbuffer, gps_CRC16_checksum(superbuffer));
  
  radio1.write(0x07, 0x08); // turn tx on
  rtty_txstring("$$$$");
  rtty_txstring(superbuffer);
  
}
