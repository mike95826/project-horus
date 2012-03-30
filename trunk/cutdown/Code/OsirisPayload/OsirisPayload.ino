/*
	Osiris Payload Code
	
	Authors:	Mark Jessop (mjessop<at>eleceng.adelaide<dot>edu.au)
				Joel Stanley
				
	Date: 2011-03-30
	
	
	Blah blah, GPLv3 blah blah.

*/

#include <RF22.h>
#include <SPI.h>

#define PWR_LED	A2
#define	STATUS_LED	A3
#define	ONEWIRE	3
#define WIRE_FET 8
#define VALVE_FET 7

#define TX_FREQ	431.650
#define	RX_FREQ	431.650
#define TX_POWER	RF22_TXPOW_1DBM  // Options are 1,2,5,8,11,14,17,20 dBm

// Singleton instance of the RFM22B Library 
RF22 rf22;

// Variables & Buffers
char txbuffer [128];
int	rfm_temp;
uint8_t rssi_floor = 0;
uint8_t got_msg = 0;
char relaymessage[40] = "No uplink received yet";

void setup(){
	// Setup out IO pins
	pinMode(PWR_LED, OUTPUT);
	pinMode(STATUS_LED, OUTPUT);
	pinMode(WIRE_FET, OUTPUT);
	pinMode(VALVE_FET, OUTPUT);
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
	RFM22B_RTTY_Mode();
	delay(100);
	
	rfm_temp = (rf22.temperatureRead( RF22_TSRANGE_M64_64C,0 ) / 2) - 64;
	//rfm_temp = rfm_temp - 64;
	
	sprintf(txbuffer, "    VK5QI OSIRIS Payload. %d,%d,%d,%s\n", rfm_temp,rssi_floor, got_msg,relaymessage);
	got_msg = 0;
	digitalWrite(STATUS_LED, LOW);
	rtty_txstring(txbuffer);
	delay(100);
	RFM22B_RX_Mode();
	delay(500);
	digitalWrite(STATUS_LED, HIGH);
	rssi_floor = rf22.rssiRead();
	uint8_t data[] = "READY FOR CMD";
    rf22.send(data, sizeof(data));
    rf22.waitPacketSent();
    digitalWrite(STATUS_LED, LOW);
	uint8_t buf[RF22_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
	if (rf22.waitAvailableTimeout(10000)){
		got_msg = rf22.lastRssi();
		if(rf22.recv(buf, &len)){
			for (int lc = 0; lc < 40; lc += 1){
              relaymessage[lc]=char(buf[lc]);  
            }
        }
		digitalWrite(STATUS_LED, HIGH);
		delay(1000);
		//uint8_t data[] = "OK";
		//rf22.send(data, sizeof(data));
    	//rf22.waitPacketSent();
    	alert_sound();
	}
}

void alert_sound(){
	RFM22B_RTTY_Mode();
	delay(100);
	rf22.spiWrite(0x073, 0x00);
	delay(300);
	rf22.spiWrite(0x073, 0x02);
	delay(300);
	rf22.spiWrite(0x073, 0x00);
	delay(300);
	rf22.spiWrite(0x073, 0x02);
	delay(300);
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