#include <TinyGPS.h>
#include <stdio.h>
#include <util/crc16.h>

TinyGPS gps;

int hour = 0 , minute = 0 , second = 0, numbersats = 99, n, j, q, navmode, count = 0;
unsigned long date, time, chars, age;
char latbuf[12] = "0", lonbuf[12] = "0", superbuffer [120], altbuf [12] = "0", checksum [10];
long int ialt = 123;
float flat, flon;

void setup(){
  
  Serial.begin(9600);
  pinMode(4, OUTPUT); //Radio Tx0
  pinMode(5, OUTPUT); //Radio Tx1
  pinMode(6, OUTPUT); //Radio En
  pinMode(8, OUTPUT); //LED
  
  digitalWrite(6, HIGH); // Enable Radio
  
  setupGPS();
}

//****************************************************************
void loop(){
   
   count++;
   
   if((count % 200) == 0) {
     navmode = 0;
     setupGPS();
   }
   Serial.println("$PUBX,00*33"); //Poll GPS
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
          
          numbersats = gps.sats();
          
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
          digitalWrite(8, LOW);
        }
      }
    
    
    
    n=sprintf (altbuf, "%05ld", ialt);
    n=sprintf (superbuffer, "$$PICO,%d,%02d:%02d:%02d,%s,%s,%ld,%d,%d", count, hour, minute, second, latbuf, lonbuf, ialt, numbersats, navmode);
   
    n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));

    rtty_txstring(altbuf);
    rtty_txstring(superbuffer);
    delay(100);
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
                    
                    digitalWrite(4, HIGH);  
                    digitalWrite(5, LOW);
		}
		else
		{
		  // low
                    
                    digitalWrite(5, HIGH);
                    digitalWrite(4, LOW);
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
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  delay(1000);
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  delay(1000);
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
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

