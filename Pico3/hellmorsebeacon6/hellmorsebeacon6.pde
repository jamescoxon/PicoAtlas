#include <SPI.h>
//#include "TinyGPS.h"

#define RF22_REG_07_OPERATING_MODE1             0x07
#define RF22_SWRES                              0x80
#define RF22_SPI_WRITE_MASK                     0x80
#define RF22_REG_0B_GPIO_CONFIGURATION0         0x0b
#define RF22_REG_0C_GPIO_CONFIGURATION1         0x0c
#define RF22_SBSEL                              0x40
#define RF22_HBSEL                              0x20
#define RF22_REG_73_FREQUENCY_OFFSET1           0x73
#define RF22_REG_74_FREQUENCY_OFFSET2           0x74
#define RF22_REG_75_FREQUENCY_BAND_SELECT       0x75
#define RF22_REG_76_NOMINAL_CARRIER_FREQUENCY1  0x76
#define RF22_REG_77_NOMINAL_CARRIER_FREQUENCY0  0x77
#define RF22_FREQERR                            0x08
#define RF22_REG_02_DEVICE_STATUS               0x02
#define RF22_REG_6D_TX_POWER                    0x6d

// RF22_REG_6D_TX_POWER                         0x6d
#define RF22_TXPOW                              0x07
#define RF22_TXPOW_4X31                         0x08 // Not used in RFM22B
#define RF22_TXPOW_1DBM                         0x00
#define RF22_TXPOW_2DBM                         0x01
#define RF22_TXPOW_5DBM                         0x02
#define RF22_TXPOW_8DBM                         0x03
#define RF22_TXPOW_11DBM                        0x04
#define RF22_TXPOW_14DBM                        0x05
#define RF22_TXPOW_17DBM                        0x06
#define RF22_TXPOW_20DBM                        0x07

#define RF22_REG_1C_IF_FILTER_BANDWIDTH         0x1c
#define RF22_REG_1F_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE   0x1f
#define RF22_REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE    0x20
#define RF22_REG_1C_IF_FILTER_BANDWIDTH                 0x1c
#define RF22_REG_2C_OOK_COUNTER_VALUE_1                 0x2c
#define RF22_REG_58_CHARGE_PUMP_CURRENT_TRIMMING        0x58
#define RF22_REG_69_AGC_OVERRIDE1                       0x69
#define RF22_REG_6E_TX_DATA_RATE1                       0x6e
#define RF22_REG_21_CLOCK_RECOVERY_OFFSET2              0x21
#define RF22_REG_22_CLOCK_RECOVERY_OFFSET1              0x22
#define RF22_REG_23_CLOCK_RECOVERY_OFFSET0              0x23
#define RF22_REG_24_CLOCK_RECOVERY_TIMING_LOOP_GAIN1    0x24
#define RF22_REG_25_CLOCK_RECOVERY_TIMING_LOOP_GAIN0    0x25
#define RF22_REG_2D_OOK_COUNTER_VALUE_2                 0x2d
#define RF22_REG_2E_SLICER_PEAK_HOLD                    0x2e
#define RF22_REG_6F_TX_DATA_RATE0                       0x6f
#define RF22_REG_70_MODULATION_CONTROL1                 0x70
#define RF22_REG_71_MODULATION_CONTROL2                 0x71
#define RF22_REG_72_FREQUENCY_DEVIATION                 0x72

// These register masks etc are named wherever possible
// corresponding to the bit and field names in the RF-22 Manual
#define RF22_DEVICE_TYPE_RX_TRX                 0x08
#define RF22_DEVICE_TYPE_TX                     0x07
#define RF22_REG_00_DEVICE_TYPE                 0x00

//Initiate GPS
//TinyGPS gps;

int count = 0, j, q, n, hour = 0 , minute = 0 , second = 0;
unsigned long date, time, chars, age;
long int ialt = 123, lat = 5200, lon = 100;

//char superbuffer [100];

int select_radio = 10;
uint8_t _deviceType;

void radio_setMode(uint8_t mode)
{
    radio_spiWrite(RF22_REG_07_OPERATING_MODE1, mode);
}

void radio_reset()
{
    radio_spiWrite(RF22_REG_07_OPERATING_MODE1, RF22_SWRES);
    // Wait for it to settle
    delay(1); // SWReset time is nominally 100usec
}

uint8_t radio_spiRead(uint8_t reg)
{
    digitalWrite(select_radio, LOW);
    SPI.transfer(reg & ~RF22_SPI_WRITE_MASK); // Send the address with the write mask off
    uint8_t val = SPI.transfer(0); // The written value is ignored, reg value is read
    digitalWrite(select_radio, HIGH);
    return val;
}

void radio_spiWrite(uint8_t reg, uint8_t val)
{
    digitalWrite(select_radio, LOW);
    SPI.transfer(reg | RF22_SPI_WRITE_MASK); // Send the address with the write mask on
    SPI.transfer(val); // New value follows
    digitalWrite(select_radio, HIGH);
}

void radio_spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len)
{
    digitalWrite(select_radio, LOW);
    SPI.transfer(reg & ~RF22_SPI_WRITE_MASK); // Send the start address with the write mask off
    while (len--)
	*dest++ = SPI.transfer(0);
    digitalWrite(select_radio, HIGH);
}

void radio_spiBurstWrite(uint8_t reg, uint8_t* src, uint8_t len)
{
    digitalWrite(select_radio, LOW);
    SPI.transfer(reg | RF22_SPI_WRITE_MASK); // Send the start address with the write mask on
    while (len--)
	SPI.transfer(*src++);
    digitalWrite(select_radio, HIGH);
}

uint8_t radio_statusRead()
{
    return radio_spiRead(RF22_REG_02_DEVICE_STATUS);
}

// Returns true if centre + (fhch * fhs) is within limits
// Caution, different versions of the RF22 suport different max freq
// so YMMV
boolean setFrequency(float centre)
{
    uint8_t fbsel = RF22_SBSEL;
    if (centre < 240.0 || centre > 960.0) // 930.0 for early silicon
	return false;
    if (centre >= 480.0)
    {
	centre /= 2;
	fbsel |= RF22_HBSEL;
    }
    centre /= 10.0;
    float integerPart = floor(centre);
    float fractionalPart = centre - integerPart;

    uint8_t fb = (uint8_t)integerPart - 24; // Range 0 to 23
    fbsel |= fb;
    uint16_t fc = fractionalPart * 64000;
    radio_spiWrite(RF22_REG_73_FREQUENCY_OFFSET1, 0);  // REVISIT
    radio_spiWrite(RF22_REG_74_FREQUENCY_OFFSET2, 0);
    radio_spiWrite(RF22_REG_75_FREQUENCY_BAND_SELECT, fbsel);
    radio_spiWrite(RF22_REG_76_NOMINAL_CARRIER_FREQUENCY1, fc >> 8);
    radio_spiWrite(RF22_REG_77_NOMINAL_CARRIER_FREQUENCY0, fc & 0xff);
    return !(radio_statusRead() & RF22_FREQERR);
}

void setTxPower(uint8_t power)
{
    radio_spiWrite(RF22_REG_6D_TX_POWER, power);
}

// Sets registers from a canned modem configuration structure
//{ 0x2b, 0x03, 0xf4, 0x20, 0x41, 0x89, 0x00, 0x36, 0x40, 0x0a, 0x1d, 0x80, 0x60, 0x10, 0x62, 0x2c, 0x00, 0x08 }; // Unmodulated carrier
void setModemRegisters()
{   
    	radio_spiWrite(RF22_REG_1C_IF_FILTER_BANDWIDTH, 0x2b);
	radio_spiWrite(RF22_REG_1F_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE, 0x03);
	radio_spiWrite(RF22_REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE, 0xf4);
	radio_spiWrite(RF22_REG_21_CLOCK_RECOVERY_OFFSET2, 0x20);
	radio_spiWrite(RF22_REG_22_CLOCK_RECOVERY_OFFSET1, 0x41);
	radio_spiWrite(RF22_REG_23_CLOCK_RECOVERY_OFFSET0, 0x89);
	radio_spiWrite(RF22_REG_24_CLOCK_RECOVERY_TIMING_LOOP_GAIN1, 0x00);
	radio_spiWrite(RF22_REG_25_CLOCK_RECOVERY_TIMING_LOOP_GAIN0, 0x36);
	radio_spiWrite(RF22_REG_2C_OOK_COUNTER_VALUE_1, 0x40);
	radio_spiWrite(RF22_REG_2D_OOK_COUNTER_VALUE_2, 0x0a);
	radio_spiWrite(RF22_REG_2E_SLICER_PEAK_HOLD, 0x1d);
	radio_spiWrite(RF22_REG_58_CHARGE_PUMP_CURRENT_TRIMMING, 0x80);
	radio_spiWrite(RF22_REG_69_AGC_OVERRIDE1, 0x60);
	radio_spiWrite(RF22_REG_6E_TX_DATA_RATE1, 0x10);
	radio_spiWrite(RF22_REG_6F_TX_DATA_RATE0, 0x62);
	radio_spiWrite(RF22_REG_70_MODULATION_CONTROL1, 0x2c);
	radio_spiWrite(RF22_REG_71_MODULATION_CONTROL2, 0x00);
	radio_spiWrite(RF22_REG_72_FREQUENCY_DEVIATION, 0x08);
}

boolean radioinit()
{
    // Wait for RF22 POR (up to 16msec)
  delay(16);
  
  // Initialise the slave select pin
  pinMode(select_radio, OUTPUT);
  digitalWrite(select_radio, HIGH);
  
  // start the SPI library:
  // Note the RF22 wants mode 0, MSB first and default to 1 Mbps
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);  // (16 Mhz / 16) = 1 MHz
  
  radio_reset();

      // Get the device type and check it
    // This also tests whether we are really connected to a device
    _deviceType = radio_spiRead(RF22_REG_00_DEVICE_TYPE);
    if (   _deviceType != RF22_DEVICE_TYPE_RX_TRX
        && _deviceType != RF22_DEVICE_TYPE_TX)
	return false;

  // Ensure the antenna can be switched automatically according to transmit and receive
  // This assumes GPIO0(out) is connected to TX_ANT(in) to enable tx antenna during transmit
  // This assumes GPIO1(out) is connected to RX_ANT(in) to enable rx antenna during receive
  radio_spiWrite (RF22_REG_0B_GPIO_CONFIGURATION0, 0x12) ; // TX state
  radio_spiWrite (RF22_REG_0C_GPIO_CONFIGURATION1, 0x15) ; // RX state
  
  
  return true;

}

void setup()
{
  Serial.println("Starting");
  if(!radioinit()){
    Serial.println("Failed init");
  }
  // Defaults after init are 434.0MHz, modulation GFSK_Rb2_4Fd36
  setFrequency(434.1);
  setModemRegisters();
  setTxPower(RF22_TXPOW_11DBM);

  Serial.begin(9600);

  CWtone();   
}

void loop()
{  
    CWtone();  
    delay(2000);
}
