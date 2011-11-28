/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "config.h"
#include "ax25.h"
#include "gps.h"
#include "aprs.h"
#include "sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include "trackuino.h"
#include "math.h"
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "modem.h"

#include <Wire.h>
#include <WProgram.h>

const struct s_address addresses[] =
{ 
    {D_CALLSIGN, D_CALLSIGN_ID},  // Destination callsign
    {S_CALLSIGN, S_CALLSIGN_ID},  // Source callsign (-11 = balloon, -9 = car)
    #ifdef DIGI_PATH1
     {DIGI_PATH1, DIGI_PATH1_TTL}, // Digi1 (first digi in the chain)
    #endif
    #ifdef DIGI_PATH2
     {DIGI_PATH2, DIGI_PATH2_TTL}, // Digi2 (second digi in the chain)
    #endif
};

// Module functions
float meters_to_feet(float m)
{
  // 10000 ft = 3048 m
  return m / 0.3048;
}

// Exported functions

#ifdef APRS_TIMESTAMPED
void aprs_send()
{
  char temp[12];                   // Temperature (int/ext)
 
  ax25_send_header(addresses, sizeof(addresses)/sizeof(s_address));
  ax25_send_byte('/');                // Report w/ timestamp, no APRS messaging. $ = NMEA raw data
  // ax25_send_string("021709z");     // 021709z = 2nd day of the month, 17:09 zulu (UTC/GMT)
  ax25_send_string(gps_time);         // 170915 = 17h:09m:15s zulu (not allowed in Status Reports)
  ax25_send_byte('h');
  ax25_send_string(gps_aprs_lat);     // Lat: 38deg and 22.20 min (.20 are NOT seconds, but 1/100th of minutes)
  ax25_send_byte('/');                // Symbol table
  ax25_send_string(gps_aprs_lon);     // Lon: 000deg and 25.80 min
  ax25_send_byte('O');                // Symbol: O=balloon, -=QTH
  snprintf(temp, 4, "%03d", (int)(gps_course + 0.5)); 
  ax25_send_string(temp);             // Course (degrees)
  ax25_send_byte('/');                // and
  snprintf(temp, 4, "%03d", (int)(gps_speed + 0.5));
  ax25_send_string(temp);             // speed (knots)
  ax25_send_string("/A=");            // Altitude (feet). Goes anywhere in the comment area
  snprintf(temp, 7, "%06ld", (long)meters_to_feet(gps_altitude));
  ax25_send_string(temp);
  ax25_send_string("/Ti=");
  sprintf(temp, "%d", sensors_int_ds18b20());
  ax25_send_string(temp);
  ax25_send_string("/Te=");
  ax25_send_string(itoa(sensors_ext_ds18b20(), temp, 10));
  ax25_send_byte(' ');
  ax25_send_string(APRS_COMMENT);     // Comment
  ax25_send_footer();
  ax25_flush_frame();                 // Tell the modem to go
}
#endif

#ifdef APRS_NOTIMESTAMP
void aprs_send()
{
  char temp[12];                   // Temperature (int/ext)

  /* Complete UI Frame Header */
  ax25_send_header(addresses, sizeof(addresses)/sizeof(s_address));

  /* now complete information field. Send the APRS type identifier */
  ax25_send_byte('!');                // Report without timestamp, no APRS messaging

  /* latitude, longitude, balloon symbol */
  ax25_send_string(gps_aprs_lat);     // Lat: 38deg and 22.20 min (.20 are NOT seconds, but 1/100th of minutes)
  ax25_send_byte('/');                // Symbol table
  ax25_send_string(gps_aprs_lon);     // Lon: 000deg and 25.80 min
  ax25_send_byte('O');                // Symbol: O=balloon, -=QTH
  
  /* Course and Speed */
  snprintf(temp, 4, "%03d", (int)(gps_course + 0.5)); 
  ax25_send_string(temp);             // Course (degrees)
  ax25_send_byte('/');                // and
  snprintf(temp, 4, "%03d", (int)(gps_speed + 0.5));
  ax25_send_string(temp);             // speed (knots)
  ax25_send_string("/A=");            // Altitude (feet). Goes anywhere in the comment area

  /* Put the Altitude in the comment section */
  snprintf(temp, 7, "%06ld", (long)meters_to_feet(gps_altitude));
  ax25_send_string(temp);
  ax25_send_byte(' ');

  /* Now send any comment */
  ax25_send_string(APRS_COMMENT);     // Comment
  ax25_send_footer();
  ax25_flush_frame();                 // Tell the modem to go
}

#endif

#if defined(APRS_TELEMETRY)

/*  allocate come memory for our analog and digital telemetry values, these
    will be updated when we call the aprs_update_analog() function, just
    prior to transmission */
static uint8_t analog[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t digital = 0x00;

/*  We will define the APRS Telemetry parameters as strings, since they are sent
    as bulletins.  So we need to define the strings individually since the length
    isn't going to be fixed. We can then define an array of pointers to these
    strings which we can then clock out using a single function */
#define TELEM_DATA_PACKETS              4
static uint8_t telem_param[] PROGMEM = ":VK5ZM-11 :PARAM.Battery,ITemp,Etemp,,,Chut,,,,,,,\0";
static uint8_t telem_units[] PROGMEM = ":VK5ZM-11 :UNIT.V,degC,degC,,,Open,,,,,,,\0";
static uint8_t telem_coeff[] PROGMEM = ":VK5ZM-11 :EQNS.0,0.0129,0,0,1,-128,0,1,-128,0,0,0,0,0,0\0";
static uint8_t telem_bits[]  PROGMEM = ":VK5ZM-11 :BITS.11111111,Project Horus HAB\0";
static uint8_t *telem_data[TELEM_DATA_PACKETS] PROGMEM = { &telem_param[0],  \
                                                           &telem_units[0],  \
                                                           &telem_coeff[0],  \
                                                           &telem_bits[0] };

/* The function aprs_send_analog() will take a variable out of our analog[] array
    convert it to a string, append a comma and load this into the modem buffer.
    This function is called five times to load all of the analog telemetry values
    into our packet */
static inline void aprs_send_analog(uint8_t var)
{
  char temp[6];

  /*  get the analog value, put into our string, add a comma and move the null
      char then send */
  snprintf(temp, 4, "%03d", analog[var]);
  temp[3] = ',';
  temp[4] = 0x00;
  ax25_send_string(temp);
  return; 
}

/*  The function aprs_send_ditial() will encode the bits stored in the digital 
    variable as a string of 1's and 0's that is then loaded into the modem buffer.
    This function is called once. */
static inline void aprs_send_digital(uint8_t val)
{
  char temp[8];
  char *p;
  register uint8_t i ;
  
  p = &temp[8];  //setup pointer to string we're going to store 8 characters
  *(p--) = 0x00;    //tack a null on the end and move pointer
  
  /* work our way backwards through parameter val and populate our string
     with a series of 1's and 0's */
  for(i=0; i < 8; i++)
  {
    if( val & (1<<i))
       *p = '1';
    else
      *p = '0';
    p -= 1;
  }
  
  ax25_send_string(temp);
  return; 
}

/*  The function aprs_telemetry_data() builds the entire Telemetry packet in the
    modem buffer.  It has to include the packet header, analog telemetry values,
    digital telemetry values and footer. This function is called by the mainloop()
    when it's necessary to send telemetry packets.  It does not wait for the modem
    to finish sending data before returning. */
void aprs_telemetry_data(void)
{
  char temp[12];                   // Temperature (int/ext)
  uint8_t i;

  /*  keep a count of the telemetry sequence number, will count from 0-255
      before it rolls over */
  static uint8_t sequence = 0;

  /* load the AX25 header into the modem buffer */
  ax25_send_header(addresses, sizeof(addresses)/sizeof(s_address));

  /* now complete information field. Send the APRS type identifier and give
     it the right sequence number */
  ax25_send_byte('T');                // Report without timestamp, no APRS messaging
  ax25_send_byte('#');
  snprintf(temp, 4, "%03d", sequence);  //sequence number
  ax25_send_string(temp);
  ax25_send_byte(',');

  for(i=0; i < 5; i++)         // send out the analog values
    aprs_send_analog(i);

  aprs_send_digital(digital);  // tack on the digital bits
    
  ax25_send_footer();
  ax25_flush_frame();          // Tell the modem to go
  
  sequence += 1;               // increpement the telemetry sequence number
}

/*  The function aprs_telemetry_params() builds four Telemetry packets and sends
    them one after the other.  The four packets contain the information necessary
    for receiving stations to decode the telemetry being send from the payload.
    This function is called periodically by the mainloop(, since four rather
    large packets are sent, it is suggested that this is done every 10-15minutes.
    The function will wait for the modem to finish sending between packets, before
    it rerturns. */
void aprs_telemetry_params(void)
{
  uint8_t val;
  uint8_t cnt;
  uint8_t *p;
  uint8_t i;

  for(i=0; i < TELEM_DATA_PACKETS; i++)
  {
    /* send hearder information to modem */
    ax25_send_header(addresses, sizeof(addresses)/sizeof(s_address));

    /* fetch pointer to string from PROGMEM */
    p = (uint8_t*)pgm_read_word( &telem_data[i] );
    
    /* setup counter, make sure we are protected from buffer overrun
       should our do while loop go astray. Set counter to the maximum
       numbers we can load into an APRS bulletin */
    cnt = 67;

    /* send the string in PROGMEM to the modem */
    do
    {
      /*  fetch the characters for string from PROGMEM, if we have found
          the end of the string then terminate, otherwise send the
          char to the modem.  Make sure we exit this loop if something
          goes wrong (dont overflow buffer!) */
      val = pgm_read_byte(p++);
      if(val == 0) break;
      ax25_send_byte(val);
      cnt -= 1;
    }
    while( cnt > 0);

    /* send the footer to modem and tell the modem to flush frame */  
    ax25_send_footer();
    ax25_flush_frame();
    /* wait for the modem to send the data before loading next packet */
    while(modem_busy()) {};

  }
}

static uint8_t get_temp(int16_t val)
{
  if(val < -128)
    return 0;
  else if(val > 127)
    return 255;
  else
    return (uint8_t)(val += 128);
}

/*  The function aprs_update_analog() is called by the mainloop() prior to
    calling aprs_send_data().  This function fetches the analog and digital
    telemetry values from various places within the code and formats it
    approriately for transmission */
void aprs_update_analog(void)
{
  /*  work out the battery volts in APRS units
      V = a*v^2 + b*v + c where: a = 0, b = 0.0129, c = 0*/
  analog[0] = (analogRead(BATT_VOLTS) >> 2);    //convert to 8-bits

  /* fetch the temperatures from our Dallas one-wire sensors */
  sensors_request_ds18b20();

  analog[1] = get_temp( sensors_int_ds18b20() );    //already range checked !

  /*  work out the external temperature in APRS units
      T = a*v^2 + b*v + c where: a = 0, b = 1, c = -128 */
  analog[2] = get_temp( sensors_ext_ds18b20());  // has already been range checked !

  /* the following analog values have not yet been implemented */
  analog[3] = 0;       //BLANK
  analog[4] = 0;       //BLANK
  digital = 0x00;      //Not yet implemented
}

void aprs_setup(void)
{
    /* we will setup the analog reference */
    analogReference(DEFAULT);                //should use 3.3V rail as reference 
}

#endif
