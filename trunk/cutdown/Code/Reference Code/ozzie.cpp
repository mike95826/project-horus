// this code is shamelessly borrowed from multiple sources - especially jcoxon!
#include <SPI.h>
#include <OneWire.h>
#include <RF22.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

//#include <stdio.h>
#include <util/crc16.h>
SoftwareSerial nss(5,4); 
RF22 rf22;
TinyGPS gps;
OneWire ds(8); // DS18x20 Temperature chip i/o One-wire

//Sensor variables
byte address0[8] = {0x28, 0x5A, 0xEB, 0xC1, 0x3, 0x0, 0x0, 0x7}; //  DS18B20 28 25 55 C2 2 0 0 75
int temp0 = 0;



//General Variables
int lastRSSI;
int count = 1;
byte navmode = 99;
float flat, flon;
unsigned long date, time, chars, age;
int bstart = 3;
int bend = 9;
int bstep = 1;
char relaymessage[40] = "No uplink received yet";
int channel;  //current Receive Channel

float prefchannels[]={459.0,459.025,459.05,459.075,459.1,459.125,459.150,459.175,459.200,459.225 };
float freq;
int rssiArray[10];
int chancount;

int hour = 0 , minute = 0 , second = 0, oldsecond = 0;
char latbuf[12] = "0", lonbuf[12] = "0", altbuf[12] = "0";
long int ialt = 123;
int numbersats = 99;

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
	for (i=0;i<7;i++)
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
               
                  rf22.spiWrite(0x073, 0x00);
		}
		else
		{
		  // low
               
                  rf22.spiWrite(0x073, 0x01);
		}
                delayMicroseconds(19500); // 10000 = 100 BAUD 19500 is default
 
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
  
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    nss.write(MSG[i]);
  }
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
	uint8_t b;
	uint8_t ackByteID = 0;
	uint8_t ackPacket[10];
        nss.flush();
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
	for (uint8_t i=2; i<8; i++) {
		ackPacket[8] = ackPacket[8] + ackPacket[i];
		ackPacket[9] = ackPacket[9] + ackPacket[8];
	}
 
	while (1) {
 
		// Test for success
		if (ackByteID > 9) {
				// All packets in order!
                               Serial.println("rcvd ok");
				return true;
		}
 
		// Timeout if no valid response in 3 seconds
		if (millis() - startTime > 3000) { 
                        Serial.println("Timeout");
			return false;
		}
 
		// Make sure data is available to read
		if (nss.available()) {
			b = nss.read();

			// Check that bytes arrive in sequence as per expected ACK packet
			if (b == ackPacket[ackByteID]) { 
				ackByteID++;
                                //Serial.print(ackPacket[ackByteID], HEX);
                                //Serial.print(" ");
			} else {
				ackByteID = 0;	// Reset and look again, invalid order
			}
 
		}
	}
}

//Function to poll the NAV5 status of a Ublox GPS module (5/6)
//Sends a UBX command (requires the function sendUBX()) and waits 3 seconds
// for a reply from the module. It then isolates the byte which contains 
// the information regarding the NAV5 mode,
// 0 = Pedestrian mode (default, will not work above 12km)
// 6 = Airborne 1G (works up to 50km altitude)
//Adapted by jcoxon from getUBX_ACK() from the example code on UKHAS wiki
// http://wiki.ukhas.org.uk/guides:falcom_fsa03
boolean checkNAV(){
  uint8_t b, bytePos = 0;
  uint8_t getNAV5[] = { 0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84 }; //Poll NAV5 status
  
  nss.flush();
  unsigned long startTime = millis();
  sendUBX(getNAV5, sizeof(getNAV5)/sizeof(uint8_t));
  
  while (1) {
    // Make sure data is available to read
    if (nss.available()) {
      b = nss.read();
   //  Serial.print(b);
      if(bytePos == 8){
        navmode = b;
        return true;
      }
                        
      bytePos++;
    }
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      navmode = 0;
      return false;
    }
  }
}

void setupGPS() {
  //Turning off all GPS NMEA strings apart on the uBlox module
  nss.println("$PUBX,40,GLL,0,0,0,0*5C");
  nss.println("$PUBX,40,GGA,0,0,0,0*5A");
  nss.println("$PUBX,40,GSA,0,0,0,0*4E");
  nss.println("$PUBX,40,RMC,0,0,0,0*47");
  nss.println("$PUBX,40,GSV,0,0,0,0*59");
  nss.println("$PUBX,40,VTG,0,0,0,0*5E");
  
  delay(3000); // Wait for the GPS to process all the previous commands
  
  
  
 // Check and set the navigation mode (Airborne, 1G)
 //page 117 31.10.2
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
 Serial.println("Nasvet"); 
  if (getUBX_ACK(setNav)){Serial.println("Nasvet ok");}
  //31.20.2 
  
  uint8_t setNav2[] = {0xB5, 0x62, 0x06, 0x11, 0x02,0x00, 0x08, 0x04,0x25, 0x95};
 sendUBX(setNav2, sizeof(setNav2)/sizeof(uint8_t));
 Serial.println("Powercon");
 if (getUBX_ACK(setNav2)) {Serial.println("psu ok");}
  
}

// gets temperature data from onewire sensor network, need to supply byte address, it'll check to see what type of sensor and convert appropriately
int getTempdata(byte sensorAddress[8]) {
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole;
  byte data[12], i, present = 0;
  
  ds.reset();
  ds.select(sensorAddress);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(sensorAddress);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
 LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  
  if (sensorAddress[0] == 0x10) {
    Tc_100 = TReading * 50;    // multiply by (100 * 0.0625) or 6.25
  }
  else { 
    Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25
  }
  
  
  Whole = Tc_100 / 100;  // separate off the whole and fractional portions

  if (SignBit) // If its negative
  {
     Whole = Whole * -1;
  }
  return Whole;
}




void setup()
{
  channel=0;
  freq=434.2;
  nss.begin(9600);
  Serial.begin(9600);
  delay(1000);
  Serial.println("Loaded");
 // rfm22::initSPI();
//  radio1.init(); 
 // radio1.write(0x71, 0x00); // unmodulated carrier
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
//  radio1.write(0x0b,0x12);
//  radio1.write(0x0c,0x15);
//  radio1.setFrequency(434.201);
//  radio1.write(0x07, 0x08); // turn tx on
 // radio1.write(0x6D, 0x03);// turn tx low power 8db
   if (!rf22.init())
    Serial.println("RF22 init failed");
   rf22.setFrequency(freq);
  // rf22.setModemConfig(RF22::FSK_Rb2Fd5);
  rf22.setModemConfig(RF22::UnmodulatedCarrier);
   rf22.setTxPower(RF22_TXPOW_11DBM);
  delay(5000); // We have to wait for a bit for the GPS to boot otherwise the commands get missed
  setupGPS();
  analogReference(DEFAULT); //3V3 for mini

}

void loop() { 
   char superbuffer [120];
   char checksum [10];
   char battVS [6];  //holder for string of battery voltage
   int n;
   int temp;  // holder for TX temp from RFM22b
   float battV; //float of battery Voltage
   int rssimin;  //Minimum RSSI of scanned channels
   int lc;
 
    if((count % 10) == 0) {
    
     checkNAV();
     delay(1000);
     if(navmode != 6){
       setupGPS();
       delay(1000);
     }
     checkNAV();
     delay(1000);
     //channel scanning
     rssimin=255;  //maximum value is 225
      rf22.setModeRx();
      for (chancount = bstart; chancount < bend; chancount += bstep)
      {
         freq=prefchannels[chancount];
         rf22.setFrequency(freq);
         delay(10); // Let the freq settle
         uint8_t rssi = rf22.rssiRead();
         rssiArray[chancount]=rssi;
         Serial.print(chancount);
           Serial.print(":");
           Serial.println(rssi);
         if (rssi<rssimin) {
           rssimin=rssi; 
           channel=chancount;
           
         }
       }
    }
    nss.println("$PUBX,00*33"); //Poll GPS
  //  radio1.write(0x6D, 0x03); // turn tx power to 8db
    while (nss.available())
    {
      int c = nss.read();
      Serial.print(char(c));
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
      itoa(ialt, altbuf, 10);
    }
    }
    
    battV = 3.3*analogRead(A2)/1024; // battery directly connected to A0
   dtostrf(battV, 3, 1, battVS);

    
    temp0 = getTempdata(address0);  //from dSB1820
            
    temp = rf22.temperatureRead( RF22_TSRANGE_M64_64C,0 ) / 2;  //from RFM22
    temp = temp - 64; 
       
    
    
    n=sprintf (superbuffer, "$$OZZIE1,%d,%02d:%02d:%02d,%s,%s,%ld,%d,%d,%s,%d,%d,%s", count, hour, minute, second, latbuf, lonbuf, ialt ,  temp0, temp, battVS,channel,lastRSSI,relaymessage);
    if (n > -1){
      n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));
    //downlink
      rf22.setFrequency(434.2);
       delay(10);
      rf22.setModeTx();
       delay(10);
       rf22.setModemConfig(RF22::UnmodulatedCarrier);
     delay(10000);
      rtty_txstring(superbuffer);
      Serial.println(superbuffer);
      Serial.println(channel);
      // go to receive mode
       rf22.setModemConfig(RF22::FSK_Rb2Fd5);
       rf22.setModeRx(); 
       freq=prefchannels[channel];
       rf22.setFrequency(freq);
       uint8_t buf[RF22_MAX_MESSAGE_LEN];
       uint8_t len = sizeof(buf);
       delay(5);
       if (rf22.waitAvailableTimeout(5000))
           { 
             // Should be a message by now  
            lastRSSI=rf22.lastRssi(); 
             if (rf22.recv(buf, &len))
              { 
             for (lc = 0; lc < 40; lc += 1)
                {
              relaymessage[lc]=char(buf[lc]);  
                }
              }
           }
    }
    count++;

}