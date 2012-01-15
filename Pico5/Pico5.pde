#include <avr/sleep.h>
#include <avr/wdt.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

volatile boolean f_wdt=1;
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
}

void rtty_txbit (int bit)
{
		if (bit)
		{
		  // high
                    digitalWrite(5, HIGH);
                    digitalWrite(4, LOW);
                    digitalWrite(13, LOW); //LED  
		}
		else
		{
		  // low
                    digitalWrite(4, HIGH);
                    digitalWrite(5, LOW);
                    digitalWrite(13, HIGH); //LED
		}
		delayMicroseconds(20500); // 10000 = 100 BAUD 20150
}

void setup(){

  pinMode(2, OUTPUT); //GPS Pwr
  
  Serial.begin(9600);
  pinMode(5, OUTPUT); //Radio Tx0
  pinMode(4, OUTPUT); //Radio Tx1
  pinMode(6, OUTPUT); //Radio En
  
  digitalWrite(6, LOW);
  
  
  // CPU Sleep Modes 
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)

  //cbi( SMCR,SE );      // sleep enable, power down mode
  //cbi( SMCR,SM0 );     // power down mode
  //sbi( SMCR,SM1 );     // power down mode
  //cbi( SMCR,SM2 );     // power down mode

  //setup_watchdog(9);
}

void loop(){
    //if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    //f_wdt=0;       // reset flag
    digitalWrite(6, HIGH);
    delay(2000);
    rtty_txstring("UUUUU$$PICO,TESTTESTTEST*TEST\n");
    digitalWrite(6, LOW);
    
    //system_sleep();
  
  //}
}

//****************************************************************  
// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep() {

  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here

    sleep_disable();                     // System continues execution here when watchdog timed out 
    sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON

}

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  Serial.println(ww);


  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);


}

//****************************************************************  
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}
