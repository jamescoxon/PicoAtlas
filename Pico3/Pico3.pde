#include <OneWire.h>
#include <Narcoleptic.h>
#include "TinyGPS_3.h"
#include <stdio.h>
#include <util/crc16.h>

TinyGPS gps;
OneWire ds(A5); // DS18x20 Temperature chip i/o One-wire

//Tempsensor variables
byte address0[8] = {0x28, 0xE8, 0x89, 0xC2, 0x2, 0x0, 0x0, 0xDF}; // External DS18B20 GPS Sensor 28 E8 89 C2 2 0 0 DF
int temp0 = 0;

int count = 0, navstatus = 0, nightloop = 0;
byte navmode = 99;
float flat, flon;
unsigned long date, time, chars, age;

int hour = 0 , minute = 0 , second = 0, oldsecond = 0;
char latbuf[12] = "0", lonbuf[12] = "0";
long int ialt = 123;
int numbersats = 99;

int battV;
int txmode = 0;
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
                    digitalWrite(A1, HIGH);  
                    digitalWrite(A2, LOW);
		}
		else
		{
		  // low
                    digitalWrite(A2, HIGH);
                    digitalWrite(A1, LOW);
		}
		//delayMicroseconds(20500); // 10000 = 100 BAUD 20150
                delayMicroseconds(20000); // 10000 = 100 BAUD 20150
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

void narcSleep(int num_loop) {
     // sleeping x minutes
     // num_loop = 96 = 8 minutes
     // num)loop = 24 = 2 minutes
   int narc_sleep = 0;
   while(narc_sleep <= num_loop)
   {
     Narcoleptic.delay(5000);
     narc_sleep++;
   }
}

void setupGPS() {
    //Turning off all GPS NMEA strings apart on the uBlox module
  Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
  Serial.println("$PUBX,40,GGA,0,0,0,0*5A");
  Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
  
  delay(3000); // Wait for the GPS to process all the previous commands
  
 // Check and set the navigation mode (Airborne, 1G)   
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  navmode = getUBXNAV5();
 
  delay(500);
  
  //set GPS to Eco mode (reduces current by 4mA)
  uint8_t setEco[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x00, 0x04, 0x1D, 0x85};
  sendUBX(setEco, sizeof(setEco)/sizeof(uint8_t));
}

// gets temperature data from onewire sensor network, need to supply byte address, it'll check to see what type of sensor and convert appropriately
int getTempdata(byte sensorAddress[8]) {
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole;
  byte data[12], i, present = 0;
  
  ds.reset();
  ds.select(sensorAddress);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(sensorAddress);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
 LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  
  if (sensorAddress[0] == 0x10) {
    Tc_100 = TReading * 50;    // multiply by (100 * 0.0625) or 6.25
  }
  else { 
    Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25
  }
  
  
  Whole = Tc_100 / 100;  // separate off the whole and fractional portions

  if (SignBit) // If its negative
  {
     Whole = Whole * -1;
  }
  return Whole;
}

void setup()
{
  pinMode(A1, OUTPUT); //Radio Tx0
  pinMode(A2, OUTPUT); //Radio Tx1
  pinMode(A0, OUTPUT); //Radio En
  digitalWrite(A0, HIGH);
  Serial.begin(9600);
  
  delay(5000); // We have to wait for a bit for the GPS to boot otherwise the commands get missed
  
  setupGPS();
  
  analogReference(EXTERNAL);

}

void loop() { 
    char superbuffer [120];
    char checksum [10];
    int n;
  
    Serial.println("$PUBX,00*33"); //Poll GPS
    
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
        
        if (second == oldsecond) {
          setupGPS();
        }
        else {
          oldsecond = second;
        }
      }
    }
    
    battV = analogRead(4);
    temp0 = getTempdata(address0);
    
    numbersats = gps.sats();
    
    navstatus = gps.navstatus();
    
    if (numbersats >= 1) {
      
      //Get Position
      gps.f_get_position(&flat, &flon);
  
      //convert float to string
      dtostrf(flat, 7, 4, latbuf);
      dtostrf(flon, 7, 4, lonbuf);
      
      //just check that we are putting a space at the front of lonbuf
      if(lonbuf[0] == ' ')
      {
        lonbuf[0] = '+';
      }
      
      // +/- altitude in meters
      ialt = (gps.altitude() / 100);    
    }
    
    n=sprintf (superbuffer, "$$PICO,%d,%02d:%02d:%02d,%s,%s,%ld,%d,%d,%d;%d;%d;%d", count, hour, minute, second, latbuf, lonbuf, ialt, navstatus, numbersats, navmode, txmode, battV, temp0);
    if (n > -1){
      n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));
      if (txmode >= 1) {
        rtty_txstring("UUUU");
      }
      rtty_txstring(superbuffer);
    }
    count++;
    
    if (count < 500) {
      delay(1000);
      txmode = 0;
    }
    else {
      if (hour > 6 && hour < 19) {
        txmode = 1; //day mode
        digitalWrite(A0, LOW); //radio sleep
        Narcoleptic.delay(5000);
        Narcoleptic.delay(5000);
        Narcoleptic.delay(5000);
        Narcoleptic.delay(5000);
        digitalWrite(A0, HIGH);//radio on
        delay(5000);// wait for it to 'tune' up
      }
      
      else {
        txmode = 2; //night mode  
        digitalWrite(A0, LOW); //radio sleep
        
        if(nightloop >= 10 && numbersats > 3) { // may need to add in '&& numbersats > 3' as in theory this could be indefinitely turning the gps on and off without a lock
          nightloop = 0;
          
          //turn off GPS
          uint8_t GPSoff[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x16, 0x74};
          sendUBX(GPSoff, sizeof(GPSoff)/sizeof(uint8_t));
          
          narcSleep(96); // sleep 8 minutes
          //delay(480000); //sleep 8 minutes
          
          //turn on GPS
          uint8_t GPSon[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x09, 0x00, 0x17, 0x76};
          sendUBX(GPSon, sizeof(GPSon)/sizeof(uint8_t));
          
          narcSleep(24);  // sleep 2 minutes
          //delay(120000); // sleep 2 minutes
        }
        else {
          Narcoleptic.delay(5000);
          Narcoleptic.delay(5000);
          nightloop++;
        }
        
        digitalWrite(A0, HIGH);//radio on
        delay(5000);// wait for it to 'tune' up
      }

    }
      

}
