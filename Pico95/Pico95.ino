/*
 AVA/ATLAS 70cm/2Mtr RTTY/APRS Tracker
 
 By Anthony Stirk M0UPU / James Coxon M6JCX
 
 Latest code can be found: https://github.com/jamescoxon/APRS_Projects / https://github.com/Upuaut/APRS_Projects
 
 Thanks and credits :
 
 Interrupt Driven RTTY Code :
 Evolved from Rob Harrison's RTTY Code.
 Thanks to : 
 http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
 http://gammon.com.au/power
 Suggestion to use Frequency Shift Registers by Dave Akerman (Daveake)/Richard Cresswell (Navrac)
 Suggestion to lock variables when making the telemetry string & Compare match register calculation from Phil Heron.
 
 RFM22B Code from James Coxon http://ukhas.org.uk/guides:rfm22b 
 
 GPS Code from jonsowman and Joey flight computer CUSF
 https://github.com/cuspaceflight/joey-m/tree/master/firmware
 Big thanks to Dave Akerman!
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 See <http://www.gnu.org/licenses/>.
 */

#include <util/crc16.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <SPI.h>
#include <RF22.h>
#include <avr/pgmspace.h>
#include "ax25modem.h"
#include "geofence.h"

static const uint8_t PROGMEM _sine_table[] = {
#include "sine_table.h"
};

/* CONFIGURABLE BITS */

#define APRS_TX_INTERVAL  120000  // APRS TX Interval
#define ASCII 8          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 50     // Baud rate for use with RFM22B Max = 600
#define RADIO_FREQUENCY 434.201
#define RADIO_POWER  0x04
/*
 0x02  5db (3mW)
 0x03  8db (6mW)
 0x04 11db (12mW)
 0x05 14db (25mW)
 0x06 17db (50mW)
 0x07 20db (100mW)
 */
#define POWERSAVING      // Comment out to turn Power Saving modes off

#define RFM22B_PIN 10
#define STATUS_LED 7            // PAVA R7 Boards have an LED on PIN4
#define GPS_ENABLE 5
#define HX1_POWER  6
#define HX1_ENABLE 4
#define HX1_TXD    3

#define BAUD_RATE      (1200)
#define TABLE_SIZE     (512)
#define PREAMBLE_BYTES (25)
#define REST_BYTES     (5)

#define PLAYBACK_RATE    (F_CPU / 256)
#define SAMPLES_PER_BAUD (PLAYBACK_RATE / BAUD_RATE)
#define PHASE_DELTA_1200 (((TABLE_SIZE * 1200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_2200 (((TABLE_SIZE * 2200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_XOR  (PHASE_DELTA_1200 ^ PHASE_DELTA_2200)

char txstring[80];  
volatile static uint8_t *_txbuf = 0;
volatile static uint8_t  _txlen = 0;

int32_t lat = 514981000, lon = -530000, alt = 36000, lat_dec = 0, lon_dec =0;
uint8_t hour = 0, minute = 0, second = 0, month = 0, day = 0, lock = 0, sats = 0;
int GPSerror = 0, count = 1, n, navmode = 0, lat_int=0,lon_int=0, errorstatus, radiostatus, gpsstatus, psm_status = 0, aprs_count = 0;
int lock_count = 0;
uint8_t buf[60]; //GPS receive buffer
char comment[3];
char superbuffer [80]; //Telem string buffer
unsigned long startTime;

RF22 rf22(RFM22B_PIN);

void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  pinMode(HX1_POWER, OUTPUT);   
  pinMode(HX1_ENABLE, OUTPUT);
  pinMode(GPS_ENABLE, OUTPUT);
  pinMode(A5, OUTPUT);  
  digitalWrite(A5, HIGH);
  digitalWrite(GPS_ENABLE, LOW);
  digitalWrite(HX1_POWER, LOW);
  digitalWrite(HX1_ENABLE, LOW);
  Serial.begin(9600);
  wait(150);
  gpsPower(2); //reset the GPS
  wait(500);
  setupGPS();
  wait(500);
  setupRadio();
  digitalWrite(STATUS_LED, LOW);
  startTime = millis();
}

void loop() {
  //Regularly reset everything in case of an error
  if(count % 20 == 0){
    re_setup();
    wait(2000);
  }
  /*
  if(count % 200 == 0){
    gpsPower(2); //reset the GPS
    wait(10000);
  }
  */
  //If GPS is ON every 5 strings check the nav mode
  if(gpsstatus == 1){
    if((count % 5) == 0){
      gps_check_nav();
      if(navmode != 6){
        setGPS_DynamicModel6();
      }
        
    }
  }
  
  //************** POWERSAVING *************//
  //Regularly checks the GPS powersaving mode - don't want to miss it about to freeze up
  // In this case we check here before the potential for reseting the GPS/Radio
  gps_PSM();
  
  if(lock == 3){
    lock_count++;
  }
  else {
    lock_count = 0;
  }
  
  //If we've had a lock for 5 loops, still have more then 4 sats and power saving
  // is not turned on yet - turn it on
  if((lock_count > 5) && (sats > 4) && (psm_status == 0)){
    setGPS_PowerSaveMode();
  }
  
  //************** APRS *************//
  /*
  if((count > 10) && (sats > 4)){
    //If 2 minutes have passed send a new APRS packet and restart the timer
    if (millis() - startTime > APRS_TX_INTERVAL) {
      //Find out where we are
      geofence_location(lat,lon);
      
      //Do not send APRS data if inside the UK
      if((comment[0] != 'X') || (alt < 100)){
        //Send APRS
        send_APRS();
      }
      
      //Reset the clock
      startTime = millis();
    }
    
  }
  */
  //************** RTTY *************//
  prepData();
  
  rtty_txstring("$$");
  rtty_txstring(superbuffer);
  
  //Sometimes we might put a delay at the end of the loop


}

//************Other Functions*****************

void prepData() {
  if(gpsstatus == 1){
    //gps_check_lock();
    gps_get_position();
    gps_get_time();
  }
  count++;
  n=sprintf (superbuffer, "$$PICO,%d,%02d:%02d:%02d,%ld,%ld,%ld,%d,%d,%d,%d", count, hour, minute, second, lat, lon, alt, sats, navmode, psm_status, lock);
  n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));
}

void re_setup(){
    //ensure HX1 is turned off
    digitalWrite(HX1_POWER, LOW);
    digitalWrite(HX1_ENABLE, LOW);
    
    //Send commands to GPS
    gpsPower(1);
    
    //Reboot Radio
    digitalWrite(A5, HIGH);
    radiostatus = 0;
    wait(1000);
    setupRadio();
}

static int pointinpoly(const int32_t *poly, int points, int32_t x, int32_t y)
{
  int32_t p0, p1, l0, l1;
  int c = 0;

  /* Read the final point */
  p0 = pgm_read_dword(&poly[points * 2 - 2]);
  p1 = pgm_read_dword(&poly[points * 2 - 1]);

  for(; points; points--, poly += 2)
  {
    l0 = p0;
    l1 = p1;
    p0 = pgm_read_dword(&poly[0]);
    p1 = pgm_read_dword(&poly[1]);

    if(y < p1 && y < l1) continue;
    if(y >= p1 && y >= l1) continue;
    if(x < p0 + (l0 - p0) * (y - p1) / (l1 - p1)) continue;

    c = !c;
  }

  return(c);
}

/*
******* LIST OF COUNTRIES ******* 
  UK - NO TRANSMISSION
  Netherlands PA/CALLSIGN
  Belgium ON/CALLSIGN
  Luxembourg LX/CALLSIGN
  Switzerland HB/CALLSIGN
  Spain EA/CALLSIGN
  Portugal CT/CALLSIGN
  France F/CALLSIGN
  Germany DL/CALLSIGN
*/

int geofence_location(int32_t lat_poly, int32_t lon_poly)
{
  if(pointinpoly(UKgeofence, 9, lat_poly, lon_poly) == true)
  {
    comment[0] = 'X';
    comment[1] = 'X';
  }
  else if(pointinpoly(Netherlands_geofence, 50, lat_poly, lon_poly) == true)
  {
    comment[0] = 'P';
    comment[1] = 'A';
  }

  else if(pointinpoly(Belgium_geofence, 60, lat_poly, lon_poly) == true)
  {
    comment[0] = 'O';
    comment[1] = 'N';
  }
  
  else if(pointinpoly(Luxembourg_geofence, 11, lat_poly, lon_poly) == true)
  {
    comment[0] = 'L';
    comment[1] = 'X';
  }

  else if(pointinpoly(Switzerland_geofence, 22, lat_poly, lon_poly) == true)
  {
    comment[0] = 'H';
    comment[1] = 'B';
  }
  
  else if(pointinpoly(Spain_geofence, 29, lat_poly, lon_poly) == true)
  {
    comment[0] = 'E';
    comment[1] = 'A';
  }
  
  else if(pointinpoly(Portugal_geofence, 19, lat_poly, lon_poly) == true)
  {
    comment[0] = 'C';
    comment[1] = 'T';
  }
  
  else if(pointinpoly(France_geofence, 60, lat_poly, lon_poly) == true)
  {
    comment[0] = ' ';
    comment[1] = 'F';
  }
  
  else if(pointinpoly(Germany_geofence, 76, lat_poly, lon_poly) == true)
  {
    comment[0] = 'D';
    comment[1] = 'L';
  }

  else
  {
    comment[0] = ' ';
    comment[1] = '#';
  }
}

void tx_aprs()
{
  char slat[5];
  char slng[5];
  char stlm[9];
  static uint16_t seq = 0;
  double aprs_lat, aprs_lon;

  /* Convert the UBLOX-style coordinates to
   	 * the APRS compressed format */
  aprs_lat = 900000000 - lat;
  aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
  aprs_lon = 900000000 + lon / 2;
  aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;
  int32_t aprs_alt = alt * 32808 / 10000;


  /* Construct the compressed telemetry format */
  ax25_base91enc(stlm + 0, 2, seq);

  ax25_frame(
  APRS_CALLSIGN, APRS_SSID,
  "APRS", 0,
  //0, 0, 0, 0,
  //"WIDE1", 1, "WIDE2",1,
  "WIDE2", 1,0,0,
  "!/%s%sO   /A=%06ld|%s|%s/%s,%d,%d,%d",
  ax25_base91enc(slat, 4, aprs_lat),
  ax25_base91enc(slng, 4, aprs_lon),
  aprs_alt, stlm, comment, count, APRS_CALLSIGN, sats, navmode
    );

  seq++;
}

ISR(TIMER2_OVF_vect)
{
  static uint16_t phase  = 0;
  static uint16_t step   = PHASE_DELTA_1200;
  static uint16_t sample = 0;
  static uint8_t rest    = PREAMBLE_BYTES + REST_BYTES;
  static uint8_t byte;
  static uint8_t bit     = 7;
  static int8_t bc       = 0;

  /* Update the PWM output */
  OCR2B = pgm_read_byte(&_sine_table[(phase >> 7) & 0x1FF]);
  phase += step;

  if(++sample < SAMPLES_PER_BAUD) return;
  sample = 0;

  /* Zero-bit insertion */
  if(bc == 5)
  {
    step ^= PHASE_DELTA_XOR;
    bc = 0;
    return;
  }

  /* Load the next byte */
  if(++bit == 8)
  {
    bit = 0;

    if(rest > REST_BYTES || !_txlen)
    {
      if(!--rest)
      {
        /* Disable radio and interrupt */
        //PORTA &= ~TXENABLE;
        TIMSK2 &= ~_BV(TOIE2);

        /* Prepare state for next run */
        phase = sample = 0;
        step  = PHASE_DELTA_1200;
        rest  = PREAMBLE_BYTES + REST_BYTES;
        bit   = 7;
        bc    = 0;

        return;
      }

      /* Rest period, transmit ax.25 header */
      byte = 0x7E;
      bc = -1;
    }
    else
    {
      /* Read the next byte from memory */
      byte = *(_txbuf++);
      if(!--_txlen) rest = REST_BYTES + 2;
      if(bc < 0) bc = 0;
    }
  }

  /* Find the next bit */
  if(byte & 1)
  {
    /* 1: Output frequency stays the same */
    if(bc >= 0) bc++;
  }
  else
  {
    /* 0: Toggle the output frequency */
    step ^= PHASE_DELTA_XOR;
    if(bc >= 0) bc = 0;
  }

  byte >>= 1;
}

void ax25_init(void)
{
  /* Fast PWM mode, non-inverting output on OC2A */
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  /* Make sure radio is not enabled */
  //PORTA &= ~TXENABLE;

  /* Enable pins for output (Port A pin 4 and Port D pin 7) */
  //DDRA |= TXENABLE;
  pinMode(HX1_TXD, OUTPUT);
}

static uint8_t *_ax25_callsign(uint8_t *s, char *callsign, char ssid)
{
  char i;
  for(i = 0; i < 6; i++)
  {
    if(*callsign) *(s++) = *(callsign++) << 1;
    else *(s++) = ' ' << 1;
  }
  *(s++) = ('0' + ssid) << 1;
  return(s);
}

void ax25_frame(char *scallsign, char sssid, char *dcallsign, char dssid,
char *path1, char ttl1, char *path2, char ttl2, char *data, ...)
{
  static uint8_t frame[100];
  uint8_t *s;
  uint16_t x;
  va_list va;

  va_start(va, data);

  /* Pause while there is still data transmitting */
  while(_txlen);

  /* Write in the callsigns and paths */
  s = _ax25_callsign(frame, dcallsign, dssid);
  s = _ax25_callsign(s, scallsign, sssid);
  if(path1) s = _ax25_callsign(s, path1, ttl1);
  if(path2) s = _ax25_callsign(s, path2, ttl2);

  /* Mark the end of the callsigns */
  s[-1] |= 1;

  *(s++) = 0x03; /* Control, 0x03 = APRS-UI frame */
  *(s++) = 0xF0; /* Protocol ID: 0xF0 = no layer 3 data */

  vsnprintf((char *) s, 100 - (s - frame) - 2, data, va);
  va_end(va);

  /* Calculate and append the checksum */
  for(x = 0xFFFF, s = frame; *s; s++)
    x = _crc_ccitt_update(x, *s);

  *(s++) = ~(x & 0xFF);
  *(s++) = ~((x >> 8) & 0xFF);

  /* Point the interrupt at the data to be transmit */
  _txbuf = frame;
  _txlen = s - frame;

  /* Enable the timer and key the radio */
  TIMSK2 |= _BV(TOIE2);
  //PORTA |= TXENABLE;
}

char *ax25_base91enc(char *s, uint8_t n, uint32_t v)
{
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */

  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }

  return(s);
}

void send_APRS() {
  
  //Shutdown RFM22
  SPI.end();
  digitalWrite(A5, HIGH);
  wait(1000);
  
  ax25_init();
  digitalWrite(HX1_POWER, HIGH);
  wait(500);
  digitalWrite(HX1_ENABLE, HIGH);
  wait(1000);
  tx_aprs();
  wait(1000); //tried to reduce the length of this but packet didn't decode
  digitalWrite(HX1_ENABLE, LOW);
  wait(500 );
  digitalWrite(HX1_POWER, LOW);
  
  
  wait(500);
  setupRadio();
  
  aprs_count++;
  
}


void setupGPS() {
  //Turning off all GPS NMEA strings apart on the uBlox module
  // Taken from Project Swift (rather than the old way of sending ascii text)
  uint8_t setNMEAoff[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9};
  sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
  
  wait(1000);
  setGPS_DynamicModel6();
  wait(1000);
}

void wait(unsigned long delaytime) // Arduino Delay doesn't get CPU Speeds below 8Mhz
{
  unsigned long _delaytime=millis();
  while((_delaytime+delaytime)>=millis()){
  }
}

void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial.flush();
  Serial.write(0xFF);
  wait(100);
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}
void gps_check_lock()
{
  GPSerror = 0;
  Serial.flush();
  // Construct the request to the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
    0x07, 0x16                                                                                                                  };

  sendUBX(request, 8);
  // Get the message back from the GPS
  gps_get_data();
  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
    GPSerror = 11;
  }
  if( buf[2] != 0x01 || buf[3] != 0x06 ) {
    GPSerror = 12;
  }

  // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
  if( !_gps_verify_checksum(&buf[2], 56) ) {
    GPSerror = 13;
  }

  if(GPSerror == 0){
    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
      lock = buf[16];
    else
      lock = 0;

    sats = buf[53];
  }
  else {
    lock = 0;
  }
}

void gps_get_data()
{
  
  Serial.flush();
  // Clear buf[i]
  for(int i = 0;i<60;i++) 
  {
    buf[i] = 0; // clearing buffer  
  }
  
  int i = 0;
  unsigned long startTime = millis();

  while ((i<60) && ((millis() - startTime) < 1000) ) { 
    if (Serial.available()) {
      buf[i] = Serial.read();
      i++;
    }
  }
}

bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
  uint8_t a, b;
  gps_ubx_checksum(data, len, &a, &b);
  if( a != *(data + len) || b != *(data + len + 1))
    return false;
  else
    return true;
}

void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka,
uint8_t* ckb)
{
  *cka = 0;
  *ckb = 0;
  for( uint8_t i = 0; i < len; i++ )
  {
    *cka += *data;
    *ckb += *cka;
    data++;
  }
}

void setupRadio(){ 
  digitalWrite(A5, LOW);
  wait(1000);
  rf22.init();
  rf22.setModeTx();
  
  rf22.spiWrite(0x71, 0x00); // unmodulated carrier

  rf22.setFrequency(434.351);
  rf22.spiWrite(0x6D, RADIO_POWER);
  rf22.spiWrite(0x07, 0x08); 
  wait(1000);

}

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
                  rf22.spiWrite(0x073, 0x03);
		}
		else
		{
		  // low
                  rf22.spiWrite(0x073, 0x00);
		}
                delayMicroseconds(9750); // 10000 = 100 BAUD 20150

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
uint8_t gps_check_nav(void)
{
  uint8_t request[8] = {
    0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84};

  sendUBX(request, 8);
  
  // Get the message back from the GPS
  gps_get_data();

  // Verify sync and header bytes
  if( buf[0] != 0xB5 || buf[1] != 0x62 ){
    GPSerror = 41;
  }
  if( buf[2] != 0x06 || buf[3] != 0x24 ){
    GPSerror = 42;
  }
  // Check 40 bytes of message checksum
  if( !_gps_verify_checksum(&buf[2], 40) ) {
    GPSerror = 43;
  }

  // Return the navigation mode and let the caller analyse it
  navmode = buf[8];
}

void setGPS_DynamicModel6()
{
  int gps_set_sucess=0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                                                           };
  while(!gps_set_sucess)
  {
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
}

void gps_get_position()
{
  GPSerror = 0;
  Serial.flush();
  // Request a NAV-POSLLH message from the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
    0x0A                                                                                                              };

  sendUBX(request, 8);
  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 21;
  if( buf[2] != 0x01 || buf[3] != 0x02 )
    GPSerror = 22;

  if( !_gps_verify_checksum(&buf[2], 32) ) {
    GPSerror = 23;
  }

  if(GPSerror == 0) {
    // 4 bytes of longitude (1e-7)
    lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
      (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;

    lon_int=abs(lon/10000000);
    lon_dec=(labs(lon) % 10000000)/100;
    // 4 bytes of latitude (1e-7)
    lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
      (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;

    lat_int=abs(lat/10000000);
    lat_dec=(labs(lat) % 10000000)/100;


    // 4 bytes of altitude above MSL (mm)
    alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
      (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
    alt /= 1000; // Correct to meters
  }

}
void gps_get_time()
{
  GPSerror = 0;
  Serial.flush();
  // Send a NAV-TIMEUTC message to the receiver
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
    0x22, 0x67                                                                                                            };

  sendUBX(request, 8);
  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 31;
  if( buf[2] != 0x01 || buf[3] != 0x21 )
    GPSerror = 32;

  if( !_gps_verify_checksum(&buf[2], 24) ) {
    GPSerror = 33;
  }

  if(GPSerror == 0) {
    if(hour > 23 || minute > 59 || second > 59)
    {
      GPSerror = 34;
    }
    else {
      hour = buf[22];
      minute = buf[23];
      second = buf[24];
    }
  }
}
void setGPS_PowerSaveMode() {
  // Power Save Mode 
  uint8_t setPSM[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92}; // Setup for Power Save Mode (Default Cyclic 1s)
  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
  psm_status=1;
}
void setGps_MaxPerformanceMode() {
  //Set GPS for Max Performance Mode
  uint8_t setMax[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91}; // Setup for Max Power Mode
  sendUBX(setMax, sizeof(setMax)/sizeof(uint8_t));
  psm_status=0;
}

void gpsPower(int i){
  if(i == 0){
    //turn off GPS
    //  uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
    uint8_t GPSoff[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x16, 0x74};
    sendUBX(GPSoff, sizeof(GPSoff)/sizeof(uint8_t));
    gpsstatus = 0;
  }
  else if (i == 1){
    //turn on GPS
     //uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
    uint8_t GPSon[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x09, 0x00, 0x17, 0x76};
    sendUBX(GPSon, sizeof(GPSon)/sizeof(uint8_t));
    gpsstatus = 1;
    psm_status=0;
    wait(1000);
    setupGPS();
  }
  else if( i == 2 ){
    //Reset GPS rather then turn it off or on
    uint8_t set_reset[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5};
    sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
    gpsstatus = 1;
    psm_status=0;
  }
  else {
  }
}

void gps_PSM(){
  
  gps_check_lock();
  
  if(psm_status == 1){
    
    if((lock == 0) || (sats < 5)){
      setGps_MaxPerformanceMode();
      wait(1000);
      lock_count = 0;
    }
  }
}






