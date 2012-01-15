#include <SPI.h>
#include <RF22.h>
#include "TinyGPS.h"

// Singleton instance of the radio
RF22 rf22;

//Initiate GPS
TinyGPS gps;

int count = 0, j, q, n, hour = 0 , minute = 0 , second = 0;
unsigned long date, time, chars, age;
long int ialt = 123, lat = 5200, lon = 100;

char superbuffer [100];
  
struct t_htab { char c; int hellpat[5]; } ;

struct t_htab helltab[] = {

  {'1', { B00000100, B00000100, B01111100, B00000000, B00000000 } },
  {'2', { B01001000, B01100100, B01010100, B01001100, B01000000 } },
  {'3', { B01000100, B01000100, B01010100, B01010100, B00111100 } },
  {'4', { B00011100, B00010000, B00010000, B01111100, B00010000 } },
  {'5', { B01000000, B01011100, B01010100, B01010100, B00110100 } },
  {'6', { B00111100, B01010010, B01001010, B01001000, B00110000 } },
  {'7', { B01000100, B00100100, B00010100, B00001100, B00000100 } },
  {'8', { B01101100, B01011010, B01010100, B01011010, B01101100 } },
  {'9', { B00001000, B01001010, B01001010, B00101010, B00111000 } },
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
  {'P', { B01000100, B01111100, B01010100, B00010100, B00010000 } },
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
  {'-', { B00010000, B00010000, B00010000, B00010000, B00010000 } },
  {'+', { B00010000, B00010000, B01111100, B00010000, B00010000 } }

};

#define N_HELL  (sizeof(helltab)/sizeof(helltab[0]))

void helldelay()
{
  delay(8);
  //delayMicroseconds(900);
  delayMicroseconds(160);
}


void on()
{
  rf22.setModeTx(); 
  helldelay();
  rf22.setModeIdle(); 
}

void hellsend(char c)
{
  int i ;
  if (c == ' ') {
    //Serial.print(c) ;
    delay(114);
    //delayMicroseconds(282);
    return ;
  }
  for (i=0; i<N_HELL; i++) {
    if (helltab[i].c == c) {
     // Serial.print(helltab[i].c) ;
      
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
  //Serial.print("?") ;
}

void hellsendmsg(char *str)
{
  while (*str)
    hellsend(*str++) ;
  //Serial.println("");
}

void CWtone()
{
  rf22.setModeTx(); 
  delay(2000);
  rf22.setModeIdle();  
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

void setup()
{
  rf22.init();
  // Defaults after init are 434.0MHz, modulation GFSK_Rb2_4Fd36
  rf22.setFrequency(434.1);
  rf22.setModemConfig(RF22::UnmodulatedCarrier);
  rf22.setTxPower(RF22_TXPOW_11DBM);

  Serial.begin(9600);
 
  setupGPS();
  
  rf22.setModeTx(); 
  delay(2000);
  rf22.setModeIdle();   
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
    
    n=sprintf (superbuffer, "PICO,%d,%02d:%02d:%02d,%d,%d,%ld", count, hour, minute, second, lat, lon, ialt);

    hellsendmsg(superbuffer);
    delay(1000);
}
