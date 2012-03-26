//#include <NewSoftSerial.h>
#include <math.h>
#include <stdio.h>

//NewSoftSerial mySerial(12, 11);

double flightlat = 54.6506, flightlon = -6.7523;
double declatitude, declongitude;
int n, receiveddata = 0, pin = 13;
long int alt = 29520;
char superbuffer [100], checksum [5];
char NS = 'N', EW = 'W';
int firstDigit = 0, secondDigit = 0, thirdDigit = 0, fourthDigit = 0, count = 0;
unsigned long millistime = 0;
char latbuf[12], lonbuf[14];

int long time = 95133;

double fr, in;

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

void setup()
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  Serial.begin(9600);
  pinMode(2, INPUT); //Detect GPS
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  delay(1000);
  digitalWrite(pin, LOW);
}

void loop()
{ 
  while(digitalRead(2) == HIGH)
  {
    digitalWrite(pin, HIGH);
    delay(1000);
    time++;
    
    declatitude = flightlat;
    declongitude = flightlon;
    
    if(declatitude < 0) {
      declatitude = declatitude * -1;
    }
    if(declongitude < 0) {
      declongitude = declongitude * -1;
    }
    
    declatitude = fabs(declatitude) / 90.0 + (alt / 100 % 100);
    declongitude = fabs(declongitude) / 180.0 + (alt % 100);
    declatitude *= 90.0 / 99.0;
    declongitude *= 180.0 / 99.0;
    
    
    // Convert the encoded coordinates to the NMEA format
    fr = modf(declatitude, &in);
    declatitude = in * 100.0 + (fr / (100.0 / 60.0) * 100.0);
    fr = modf(declongitude, &in);
    declongitude = in * 100.0 + (fr / (100.0 / 60.0) * 100.0);
    // Round to three decimal places
    declatitude = round(declatitude * 1000.0) / 1000.0;
    declongitude = round(declongitude * 1000.0) / 1000.0;
    
    
    dtostrf(declatitude, 8, 3, latbuf);
    if(latbuf[0] == ' '){
      latbuf[0] = '0';
    }
    if(latbuf[1] == ' '){
      latbuf[1] = '0';
    }
    if(latbuf[2] == ' '){
      latbuf[2] = '0';
    }
    dtostrf(declongitude, 9, 3, lonbuf);
    
    if(lonbuf[0] == ' '){
      lonbuf[0] = '0';
    }
    if(lonbuf[1] == ' '){
      lonbuf[1] = '0';
    }
    if(lonbuf[2] == ' '){
      lonbuf[2] = '0';
    }
    if (count == 0)
    {
    delay(4000);
    Serial.println("$GPGGA,035921.905,4329.9279,N,08028.6731,W,1,04,03.5,314.6,M,-36.8,M,,*54");
    Serial.println("$GPGSA,A,3,03,06,20,31,,,,,,,,,13.3,3.5,12.9*3A");
    }
    else {
      delay(1000);
    Serial.println("$GPGGA,035921.905,4329.9279,N,08028.6731,W,1,04,03.5,314.6,M,-36.8,M,,*54");
    Serial.println("$GPGSA,A,3,03,06,20,31,,,,,,,,,13.3,3.5,12.9*3A");
    }
    //delay(1000);
    count++;
    millistime = millis();
    receiveddata++;
  }
  digitalWrite(pin, LOW);
  delay(50);
}
