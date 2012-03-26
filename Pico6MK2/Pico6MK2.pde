#include <Wire.h>
#include <SPI.h>
#include <RFM22.h>
#include "TinyGPS.h"

//Setup radio on SPI with NSEL on pin 10
rfm22 radio1(10);

//Initiate GPS
TinyGPS gps;

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

//Functions

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
  if(slowfeld == 1){
  //Slow
  delay(64);
  delayMicroseconds(900);
  }
  else
  {
  //Feld-Hell
  delay(8);
  delayMicroseconds(160);
  }

}

void on()
{
  radio1.write(0x07, 0x08); //on
  helldelay();
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

//************ BMP085 *****************

void bmp085_read_temperature_and_pressure(int* temperature, long* pressure) {
  int  ut= bmp085_read_ut();
  long up = bmp085_read_up();
   long x1, x2, x3, b3, b5, b6, p;
   unsigned long b4, b7;

   //calculate the temperature
   x1 = ((long)ut - ac6) * ac5 >> 15;
   x2 = ((long) mc << 11) / (x1 + md);
   b5 = x1 + x2;
   *temperature = (b5 + 8) >> 4;
   
   //calculate the pressure
   b6 = b5 - 4000;
   x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
   x2 = ac2 * b6 >> 11;
   x3 = x1 + x2;
   b3 = (((int32_t) ac1 * 4 + x3)<<oversampling_setting + 2) >> 2;
   x1 = ac3 * b6 >> 13;
   x2 = (b1 * (b6 * b6 >> 12)) >> 16;
   x3 = ((x1 + x2) + 2) >> 2;
   b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
   b7 = ((uint32_t) up - b3) * (50000 >> oversampling_setting);
   p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
   
   x1 = (p >> 8) * (p >> 8);
   x1 = (x1 * 3038) >> 16;
   x2 = (-7357 * p) >> 16;
   *pressure = p + ((x1 + x2 + 3791) >> 4);

}

unsigned int bmp085_read_ut() {
  write_register(0xf4,0x2e);
  delay(5); //longer than 4.5 ms
  return read_int_register(0xf6);
}

void  bmp085_get_cal_data() {
  //Serial.println("Reading Calibration Data");
  ac1 = read_int_register(0xAA);
  //Serial.print("AC1: ");
  //Serial.println(ac1,DEC);
  ac2 = read_int_register(0xAC);
  //Serial.print("AC2: ");
  //Serial.println(ac2,DEC);
  ac3 = read_int_register(0xAE);
  //Serial.print("AC3: ");
  //Serial.println(ac3,DEC);
  ac4 = read_int_register(0xB0);
  //Serial.print("AC4: ");
  //Serial.println(ac4,DEC);
  ac5 = read_int_register(0xB2);
  //Serial.print("AC5: ");
  //Serial.println(ac5,DEC);
  ac6 = read_int_register(0xB4);
  //Serial.print("AC6: ");
  //Serial.println(ac6,DEC);
  b1 = read_int_register(0xB6);
  //Serial.print("B1: ");
  //Serial.println(b1,DEC);
  b2 = read_int_register(0xB8);
  //Serial.print("B2: ");
  //Serial.println(b1,DEC);
  mb = read_int_register(0xBA);
  //Serial.print("MB: ");
  //Serial.println(mb,DEC);
  mc = read_int_register(0xBC);
  //Serial.print("MC: ");
  //Serial.println(mc,DEC);
  md = read_int_register(0xBE);
  //Serial.print("MD: ");
  //Serial.println(md,DEC);
}


long bmp085_read_up() {
  write_register(0xf4,0x34+(oversampling_setting<<6));
  delay(pressure_waittime[oversampling_setting]);
  
  unsigned char msb, lsb, xlsb;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.send(0xf6);  // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 3); // read a byte
  while(!Wire.available()) {
    // waiting
  }
  msb = Wire.receive();
  while(!Wire.available()) {
    // waiting
  }
  lsb |= Wire.receive();
  while(!Wire.available()) {
    // waiting
  }
  xlsb |= Wire.receive();
  return (((long)msb<<16) | ((long)lsb<<8) | ((long)xlsb)) >>(8-oversampling_setting);
}

void write_register(unsigned char r, unsigned char v)
{
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.send(r);
  Wire.send(v);
  Wire.endTransmission();
}

char read_register(unsigned char r)
{
  unsigned char v;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.send(r);  // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 1); // read a byte
  while(!Wire.available()) {
    // waiting
  }
  v = Wire.receive();
  return v;
}

int read_int_register(unsigned char r)
{
  unsigned char msb, lsb;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.send(r);  // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 2); // read a byte
  while(!Wire.available()) {
    // waiting
  }
  msb = Wire.receive();
  while(!Wire.available()) {
    // waiting
  }
  lsb = Wire.receive();
  return (((int)msb<<8) | ((int)lsb));
}

//************Other Functions*****************

void bmp085_read_temperature_and_pressure(int& temperature, long& pressure);

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
  
  radio1.write(0x71, 0x00); // unmodulated carrier
  
  radio1.setFrequency(434.200);
  
  radio1.write(0x07, 0x08); // turn tx on
  delay(1000);
  radio1.write(0x07, 0x01); // turn tx off
  
}

//****************Main Program****************
void setup() {
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  Serial.begin(9600);
  
  Wire.begin();
  bmp085_get_cal_data();
  
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

  bmp085_read_temperature_and_pressure(&temperature,&pressure);

  n=sprintf (superbuffer, "PICO,%d,%02d:%02d:%02d,%ld,%ld,%ld,%d,%ld", count, hour, minute, second, lat, lon, ialt, temperature, pressure);
  n = sprintf (superbuffer, "%s*%02X\n", superbuffer, gps_checksum(superbuffer));
  
  if(slowfeld == 0)
  {
    slowfeld = 1;
  }
  else {
    slowfeld = 0;
  }
  
  if(lat != 0){
    if(radioOn == 0){
      setupRadio();
      radioOn = 1;
    }
    
    hellsendmsg(superbuffer);
    radio1.write(0x07, 0x01); //make sure radio is not txing
    delay(1000);
  }
  else{
    delay(1000);
  }
  
}
