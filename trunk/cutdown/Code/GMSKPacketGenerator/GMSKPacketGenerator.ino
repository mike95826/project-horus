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
#include <avr/pgmspace.h>

#define PWR_LED	A2
#define	STATUS_LED	A3
#define	ONEWIRE	3
#define WIRE_FET 8
#define VALVE_FET 7
#define BATT	7

#define TX_FREQ	431.650
#define RX_FREQ	431.650
#define TX_POWER	RF22_TXPOW_1DBM  // Options are 1,2,5,8,11,14,17,20 dBm
#define RTTY_DELAY	19500 // 50 baud
// Singleton instance of the RFM22B Library 
RF22 rf22;




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
	delay(1000);
}

void loop(){
	RFM22B_RTTY_Mode();
	rtty_txstring("VK5QI TESTING\n");
	delay(500);
	RFM22B_RX_Mode();
	delay(1000);
/*
	uint8_t data[] = "$0 This is the longest possible message.";
	rf22.send(data, sizeof(data));
	rf22.waitPacketSent();
	delay(1000);
*/	
	uint8_t data2[] = "$0 VK5ALX";
	rf22.send(data2, sizeof(data2));
	rf22.waitPacketSent();
	delay(1000);
/*
	uint8_t data3[] = "$0 VK5ALX";
	rf22.send(data3, sizeof(data3));
	rf22.waitPacketSent();
	delay(1000);
	
	uint8_t data4[] = "$1 VK5QI 1s";
	rf22.send(data4, sizeof(data4));
	rf22.waitPacketSent();
	delay(1000);
	
	uint8_t data5[] = "$3 VK5QI 3s";
	rf22.send(data5, sizeof(data5));
	rf22.waitPacketSent();
	delay(1000);
*/
//	delay(5000);
}

// Blink both LEDs simultaneously and fast, to indicate a failure.
void payload_fail(){
	while(1){
		digitalWrite(PWR_LED, HIGH);
		digitalWrite(STATUS_LED, LOW);
		delay(300);
		digitalWrite(PWR_LED, LOW);
		digitalWrite(STATUS_LED, HIGH);
		delay(300);
	}
}