/*
	Osiris Payload Code
	
	Authors:	Mark Jessop (mjessop<at>eleceng.adelaide<dot>edu.au)
				Joel Stanley
				
	Date: 2011-03-30
	
	
	Blah blah, GPLv3 blah blah.

*/

#include <RF22.h>
#include <SPI.h>
#include <util/crc16.h>

#define PWR_LED	A2
#define	STATUS_LED	A3
#define	ONEWIRE	3
#define WIRE_FET 8
#define VALVE_FET 7
#define BATT	7

#define TX_FREQ	431.650
#define	RX_FREQ	431.650
#define TX_POWER	RF22_TXPOW_1DBM  // Options are 1,2,5,8,11,14,17,20 dBm
#define LISTEN_TIME	5000

// Singleton instance of the RFM22B Library 
RF22 rf22;

// Variables & Buffers
char txbuffer [128];
int	rfm_temp;
int rssi_floor = 0;
int last_rssi = 0;
char relaymessage[40] = "No uplink received yet";
int batt_mv = 0;
unsigned int count = 0;
unsigned int rx_count = 0;

void setup(){
	// Setup out IO pins
	pinMode(PWR_LED, OUTPUT);
	pinMode(STATUS_LED, OUTPUT);
	pinMode(WIRE_FET, OUTPUT);
	pinMode(VALVE_FET, OUTPUT);
	analogReference(INTERNAL);
	digitalWrite(PWR_LED, HIGH);
	digitalWrite(STATUS_LED, LOW);
	digitalWrite(WIRE_FET, LOW);
	digitalWrite(VALVE_FET, LOW);
	
	// Attempt to start the radio. If fail, blink.
	if(!rf22.init()) payload_fail();
	rf22.setTxPower(TX_POWER);
  	RFM22B_RTTY_Mode();
  
}

void loop(){
	count++;
	RFM22B_RTTY_Mode();
	delay(100);
	
	rfm_temp = (rf22.temperatureRead( RF22_TSRANGE_M64_64C,0 ) / 2) - 64;
	batt_mv = analogRead(BATT) * 12;
	
	sprintf(txbuffer, "$$$OSIRIS,VK5QI,%d,%d,%d,%d,%d,%s", count,rfm_temp,batt_mv,rssi_floor,last_rssi,relaymessage);
	sprintf(txbuffer, "%s*%04X\n", txbuffer, gps_CRC16_checksum(txbuffer));
	digitalWrite(STATUS_LED, LOW);
	rtty_txstring(txbuffer);
	delay(100);
	
	RFM22B_RX_Mode();
	delay(400);
	
	digitalWrite(STATUS_LED, HIGH);
	
	rssi_floor = ((int)rf22.rssiRead()*51 - 12400)/100; // Linear approximation to the graph in the datasheet.
	
	uint8_t data[] = "READY FOR CMD";
    rf22.send(data, sizeof(data));
    rf22.waitPacketSent();
    digitalWrite(STATUS_LED, LOW);
    
    
	uint8_t buf[RF22_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
	if (rf22.waitAvailableTimeout(LISTEN_TIME)){
		last_rssi = ((int)rf22.lastRssi()*51 - 12400)/100;
		if(rf22.recv(buf, &len)){
			for (int lc = 0; lc < 40; lc += 1){
              relaymessage[lc]=char(buf[lc]);  
            }
        }
		digitalWrite(STATUS_LED, HIGH);
		delay(1000);
    	alert_sound(3);
	}
}

void alert_sound(int loops){
	RFM22B_RTTY_Mode();
	delay(100);
	for(int i = 0; i<loops; i++){
		rf22.spiWrite(0x073, 0x00);
		delay(250);
		rf22.spiWrite(0x073, 0x02);
		delay(250);
	}
}

// Blink both LEDs simultaneously and fast, to indicate a failure.
void payload_fail(){
	while(1){
		digitalWrite(PWR_LED, HIGH);
		digitalWrite(STATUS_LED, HIGH);
		delay(300);
		digitalWrite(PWR_LED, LOW);
		digitalWrite(STATUS_LED, LOW);
		delay(300);
	}
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
