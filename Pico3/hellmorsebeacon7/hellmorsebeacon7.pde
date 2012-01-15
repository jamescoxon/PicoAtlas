#include <SPI.h>
#include <RFM22.h>
#include "TinyGPS.h"

rfm22 radio1(10); // radio 1 with NSEL on pin 10

int i = 0;

//Initiate GPS
TinyGPS gps;

int count = 0, j, q, n, hour = 0 , minute = 0 , second = 0;
unsigned long date, time, chars, age;
long int ialt = 123, lat = 5200, lon = 100;

char latbuf[12] = "0", lonbuf[12] = "0", countbuf[12] = "0";

//char superbuffer [60];

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
  {'/', { B01000000, B00100000, B00010000, B00001000, B00000100 } }

};

#define N_HELL  (sizeof(helltab)/sizeof(helltab[0]))

void helldelay()
{
  delay(8);
  delayMicroseconds(160);
}


void on()
{
  radio1.write(0x07, 0x08);
  helldelay();
  radio1.write(0x07, 0x01);
}

void hellsend(char c)
{
  int i ;
  if (c == ' ') {
    Serial.print(c) ;
    delay(65);
    delayMicroseconds(280);
    return ;
  }
  for (i=0; i<N_HELL; i++) {
    if (helltab[i].c == c) {
      Serial.print(helltab[i].c) ;
      
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
      delay(114);
      //delayMicroseconds(68);
      return ;
    }
  }
  /* if we drop off the end, then we send a space */
  Serial.print("?") ;
}

void hellsendmsg(char *str)
{
  while (*str)
    hellsend(*str++) ;
  Serial.println("");
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

void setup() {
  Serial.begin(9600);
  rfm22::initSPI();

  radio1.init();
  
  radio1.write(0x71, 0x00); // unmodulated carrier
  
  setupGPS();
  
  radio1.write(0x07, 0x08);
  delay(1000);
  radio1.write(0x07, 0x01);
}

void loop()
{  
     count++;
   
   if((count % 200) == 0) {
     setupGPS();
   }
   
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
          
          //Get Position
          gps.get_position(&lat, &lon, &age);
          
          // +/- altitude in meters
          ialt = (gps.altitude() / 100);   
        }
      }
    
    itoa(count, countbuf, 10);

    //n=sprintf (superbuffer, "PICO,%d,%02d:%02d:%02d,%ld,%ld,%ld", count, hour, minute, second, lat, lon, ialt);
    //hellsendmsg(superbuffer);
    
    hellsendmsg("PICO,");
    hellsendmsg(countbuf);

    delay(1000);
}
