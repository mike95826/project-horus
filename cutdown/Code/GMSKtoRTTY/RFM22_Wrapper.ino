/*
	Osiris Payload Code - RFM22B Wrapper Functions
	
	Authors:	Mark Jessop (mjessop<at>eleceng.adelaide<dot>edu.au)
				Joel Stanley
				
	Date: 2011-03-30
	
	
	Blah blah, GPLv3 blah blah.

*/


RF22::ModemConfig GMSK_500bd=  { 0x2b, 0x03, 0xd0, 0xe0, 0x10, 0x62,0x00, 0x05, 0x40, 0x0a, 0x1d, 0x80, 0x60, 0x04, 0x19, 0x2c, 0x23, 0x01 };// these values correspond to 500bps tx rate no manchestger OOK - and are the lowest values that are
//stable on the RFM22b and give decent reception.
//RF22::ModemConfig GMSK_500bd=  { 0x2b, 0x03, 0xd0, 0xe0, 0x10, 0x62,0x00, 0x05, 0x40, 0x0a, 0x1d, 0x80, 0x60, 0x01, 0xA4, 0x2c, 0x23, 0x01 };


void RFM22B_RTTY_Mode(){
	rf22.setFrequency(TX_FREQ);
	rf22.setModeTx();
	rf22.setModemConfig(RF22::UnmodulatedCarrier);
	rf22.spiWrite(0x073, 0x03); // Make sure we're on the high tone when we start
	

}


void RFM22B_RX_Mode(){
	rf22.setModeIdle(); // For some reason we need to go into an idle mode before switching to RX mode.
	rf22.setModemRegisters(&GMSK_500bd);
	rf22.setModeRx(); 
	rf22.setFrequency(RX_FREQ);
}

