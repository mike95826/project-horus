#include <RF22.h>
#include <SPI.h>
#include <string.h>
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
#define LISTEN_TIME	500
#define RTTY_DELAY	19500 // 50 baud
//#define RTTY_DELAY	3400 // 300 baud

// Singleton instance of the RFM22B Library 
RF22 rf22;

float FREQ = 431.650;

// Variables & Buffers
char txbuffer [128];
int	rfm_temp;
int rssi_floor = 0;
int last_rssi = 0;
char relaymessage[40] = "Nothing set";
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
	Serial.begin(9600);
	Serial.println("Booting...");
	
	// Attempt to start the radio. If fail, blink.
	if(!rf22.init()) payload_fail();
	rf22.setTxPower(TX_POWER);
	RFM22B_RTTY_Mode();
	Serial.println("Transmitter is up.");
}

void loop(){
	while(Serial.available()>0){ Serial.read();} // Flush the input buffer
	Serial.println("");
	Serial.println("CHOOSE WISELY:");
	Serial.println("Change (f)requency.");
	Serial.println("Set (m)essage.");
	Serial.println("(s)end message with RTTY preamble.");
	Serial.println("(i)mmediate send.");
	Serial.println("Turn reference tone (o)ff.");
	Serial.println("Turn reference tone o(n).");
	Serial.println("Go to interactive (l)isten mode.");
	
	while(Serial.available()==0){} // Wait for input
	
	char cmd = Serial.read();
	delay(300);
	while(Serial.available()>0){ Serial.read();} // Flush the input buffer
	Serial.println("");
	switch(cmd){
		case 'f':
			set_frequency();
			break;
			
		case 'm':
			read_message();
			break;
		
		case 's':
			send_message();
			break;
			
		case 'i':
			send_message_immediate();
			break;
			
		case 'o':
			RFM22B_RX_Mode();
			Serial.println("Done");
			break;
			
		case 'n':
			RFM22B_RTTY_Mode();
			Serial.println("Done");
			break;
			
		case 'l':
			interactive_listen();
			break;
		
		default:
			break;
	}
}

void read_message(){
	while(Serial.available()>0){ Serial.read();} // Flush the input buffer
	Serial.print("Enter message, then LF: ");
	int i = 0;
	while(1){
		while(Serial.available()==0){} // Wait for a character
		char temp = Serial.read();
		if(temp == '\n' || temp == '\r'){
			relaymessage[i] = 0;
			Serial.println("");
			Serial.print("New Message: ");
			Serial.println(relaymessage);
			return;
		}else{
			relaymessage[i] = temp;
		}
		i++;
	}
}

void send_message(){
	RFM22B_RTTY_Mode();
	Serial.println("Sending 10s of RTTY: .");
	rtty_txstring("VK5QI blahblahblahblahblah tune me up scotty 54321\n");
	delay(100);
	
	RFM22B_RX_Mode();
	delay(400);
	

	rf22.send((uint8_t*)relaymessage, (uint8_t)strlen(relaymessage));
	rf22.waitPacketSent();
	Serial.print("Sent: ");
	Serial.println(relaymessage);
	delay(300);
	//RFM22B_RTTY_Mode();
}

void send_message_immediate(){
	RFM22B_RX_Mode();
	delay(200);
	

	rf22.send((uint8_t*)relaymessage, (uint8_t)strlen(relaymessage));
	rf22.waitPacketSent();
	Serial.print("Sent: ");
	Serial.println(relaymessage);
	delay(300);
	//RFM22B_RTTY_Mode();
}

void set_frequency(){
	while(Serial.available()>0){ Serial.read();} // Flush the input buffer
	Serial.print("Enter Frequency in MHz (XXX.XXX): ");
	while(Serial.available()<8){}
	FREQ = Serial.parseFloat();
	rf22.setFrequency(FREQ);
	Serial.println("");
	Serial.print("Frequency set to ");
	Serial.println(FREQ,3);
}


void interactive_listen(){

	uint8_t buf[RF22_MAX_MESSAGE_LEN];
	uint8_t len = sizeof(buf);
	
	RFM22B_RX_Mode();
	Serial.println("[ and ] to tune down and up, s to send preset packet, q to quit.");
	while(1){
		int rssi_floor = ((int)rf22.rssiRead()*51 - 12400)/100;
		Serial.print(FREQ,3); Serial.print(" "); Serial.println(rssi_floor);
		if(rf22.waitAvailableTimeout(LISTEN_TIME)){
		// Heard a packet!
			if(rf22.recv(buf, &len)){
				int last_rssi = ((int)rf22.lastRssi()*51 - 12400)/100;
				Serial.print("Got Packet with RSSI "); Serial.print(last_rssi); Serial.print("dBm: ");
				Serial.write(buf,len); Serial.println("");
			}
		}
	
		while(Serial.available()>0){
			switch(Serial.read()){
				case '[':
					FREQ = FREQ - 0.001;
					rf22.setFrequency(FREQ);
					break;
				
				case ']':
					FREQ = FREQ + 0.001;
					rf22.setFrequency(FREQ);
					break;
					
				case 's':
						rf22.send((uint8_t*)relaymessage, (uint8_t)strlen(relaymessage));
						rf22.waitPacketSent();
						Serial.print("Sent: ");
						Serial.println(relaymessage);
						break;
						
				case 'q':
					return;
					break;
					
				default:
					break;
					
			}
		}
	
	}


}


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

