// rf22_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing server
// with the RF22 class. RF22 class does not provide for addressing or reliability.
// It is designed to work with the other example rf22_client

#include <SPI.h>
#include <RF22.h>

// Singleton instance of the radio
RF22 rf22;

int rx_count, lastRSSI, currentRSSI;

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
                //delayMicroseconds(9750); // 10000 = 100 BAUD 20150
                delayMicroseconds(19500); // 10000 = 100 BAUD 20150

}

void Rx_mode(){
  if (!rf22.init()){
  Serial.println("RF22 init failed");
  }
  // Defaults after init are 434.0MHz, modulation GFSK_Rb2_4Fd36
  rf22.setModemConfig(RF22::GFSK_Rb2Fd5);
  rf22.setFrequency(434.000);
  rf22.setModeRx();
}

void Tx_mode(){
  rf22.reset();
  rf22.setModeTx();
  rf22.spiWrite(0x71, 0x00); // unmodulated carrier
  rf22.setFrequency(434.100);
  rf22.spiWrite(0x6D, 0x04);// turn tx low power 11db
  rf22.spiWrite(0x07, 0x08); // turn tx on
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("Booting...");
  if (!rf22.init())
    Serial.println("RF22 init failed");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb2_4Fd36
  rf22.setModemConfig(RF22::GFSK_Rb2Fd5);
}

void loop()
{
  while (1)
  {
    
    // Wait for data over the radio, will time out after (1000*60*5)
    if(rf22.waitAvailableTimeout(300000)){
      
      // Should be a message for us now   
      uint8_t buf[RF22_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf22.recv(buf, &len))
      {
        
        //Collect Data
        rx_count++; //count number of strings rx'd
        lastRSSI = rf22.lastRssi();
        currentRSSI = rf22.rssiRead();
        
        Serial.print(rx_count);
        Serial.print(",");
        Serial.print(lastRSSI);
        Serial.print(",");
        Serial.print(currentRSSI);
        Serial.print(", rx: ");
        Serial.println((char*)buf);
        
  
        
        //Prepare to transmit data as RTTY
        Tx_mode();
        delay(5000);
        for (int i=0; i <= 3; i++){
          rtty_txstring("$$$$$");
          rtty_txstring((char*)buf);
          delay(1000);
        }
        
        //RTTY complete - now switch back to RFM22 GFSK Rx Mode
        Rx_mode();
      }
      else
      {
        Serial.println("recv failed");
      }
    }
    else{
      //Transmit own GPS location
      // 1) Turn on GPS module
      
      // 2) Get Lock
      
      // 3) Prep Data
      Tx_mode();
      
      // 4) Transmit Data (?for loop of say 5 strings)
      
      // 5) Shutdown GPS
      
      // 6) Switch back to Rx mode
      Rx_mode();
    }
  }
}

