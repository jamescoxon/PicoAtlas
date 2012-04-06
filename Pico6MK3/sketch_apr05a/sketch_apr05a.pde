//Variables
int32_t lat = 0, lon = 0, alt = 0;
uint8_t hour = 0, minute = 0, second = 0, lock = 0, sats = 0;
unsigned long startTime;
uint8_t buf[60];
int GPSerror = 0;


// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
  //Serial.println();
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

/**
 * Calculate a UBX checksum using 8-bit Fletcher (RFC1145)
 */
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

/**
 * Verify the checksum for the given data and length.
 */
bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
    uint8_t a, b;
    gps_ubx_checksum(data, len, &a, &b);
    if( a != *(data + len) || b != *(data + len + 1))
        return false;
    else
        return true;
}

/**
 * Get data from GPS, times out after 1 second.
 */
void gps_get_data()
{
    int i = 0;
    startTime = millis();
    while (1) {
    // Make sure data is available to read
    if (Serial.available()) {
      buf[i] = Serial.read();
      i++;
    }
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 1000) {
      break;
    }
    }
}
/**
 * Check the navigation status to determine the quality of the
 * fix currently held by the receiver with a NAV-STATUS message.
 */
void gps_check_lock()
{
    Serial.flush();
    // Construct the request to the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
        0x07, 0x16};
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

    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
        lock = buf[16];
    else
        lock = 0;

    sats = buf[53];
}

/**
 * Poll the GPS for a position message then extract the useful
 * information from it - POSLLH.
 */
void gps_get_position()
{
    Serial.flush();
    // Request a NAV-POSLLH message from the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
        0x0A};
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

    // 4 bytes of longitude (1e-7)
    lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
        (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;
    
    // 4 bytes of latitude (1e-7)
    lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
        (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;
    
    // 4 bytes of altitude above MSL (mm)
    alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
        (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
    alt /= 1000;

}

/**
 * Get the hour, minute and second from the GPS using the NAV-TIMEUTC
 * message.
 */
void gps_get_time()
{
    Serial.flush();
    // Send a NAV-TIMEUTC message to the receiver
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
        0x22, 0x67};
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
    
    hour = buf[22];
    minute = buf[23];
    second = buf[24];
}


void setup() {
  Serial.begin(9600);
  setupGPS();
}

void loop() {
  GPSerror = 0;
  gps_check_lock();
  
  if( lock == 0x02 || lock == 0x03 || lock == 0x04 )
  {
      gps_get_position();
      gps_get_time();
  }

  delay(5000);
  
}
