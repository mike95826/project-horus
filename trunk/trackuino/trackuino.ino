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

// Refuse to compile on arduino version 21 or lower. 22 includes an 
// optimization of the USART code that is critical for real-time operation.
#if ARDUINO < 100
#error "Oops! We need Arduino 1.0 or later"
#endif

// Trackuino custom libs
#include "aprs.h"
#include "ax25.h"
#include "buzzer.h"
#include "config.h"
#include "debug.h"
#include "gps.h"
#include "modem.h"
#include "radio.h"
#include "radio_hx1.h"
#include "radio_mx146.h"
#include "sensors.h"

// Arduino/AVR libs
#include <Wire.h>
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <avr/power.h>
#include <avr/sleep.h>

static uint32_t next_tx_millis;    // Counter for delay between position and telemetry reports

enum packets
{
  PACKET_POS,
  PACKET_TELEMETRY,
  PACKET_UNITS
};


// Private Functions
void disable_bod_and_sleep()
{
  /* This will turn off brown-out detection while
   * sleeping. Unfortunately this won't work in IDLE mode.
   * Relevant info about BOD disabling: datasheet p.44
   *
   * Procedure to disable the BOD:
   *
   * 1. BODSE and BODS must be set to 1
   * 2. Turn BODSE to 0
   * 3. BODS will automatically turn 0 after 4 cycles
   *
   * The catch is that we *must* go to sleep between 2
   * and 3, ie. just before BODS turns 0.
   */
  unsigned char mcucr;

  cli();
  mcucr = MCUCR | (_BV(BODS) | _BV(BODSE));
  MCUCR = mcucr;
  MCUCR = mcucr & (~_BV(BODSE));
  sei();
  sleep_mode();    // Go to sleep
}

void power_save()
{
  /* Enter power saving mode. SLEEP_MODE_IDLE is the least saving
   * mode, but it's the only one that will keep the UART running.
   * In addition, we need timer0 to keep track of time, timer 1
   * to drive the buzzer and timer2 to keep pwm output at its rest
   * voltage.
   */

  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();

  digitalWrite(LED_PIN, LOW);
  sleep_mode();    // Go to sleep
  digitalWrite(LED_PIN, HIGH);

  sleep_disable();  // Resume after wake up
  power_all_enable();
}

// Public Functions
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(GPS_BAUDRATE);
  #ifdef DEBUG_RESET
   Serial.println("RESET");
  #endif
  modem_setup();
  buzzer_setup();
  sensors_setup();
  gps_setup();

  #ifdef APRS_TELEMETRY
   aprs_setup();
  #endif
  
  // Schedule the next transmission within APRS_DELAY ms
  next_tx_millis = millis() + APRS_DELAY;
}

void loop()
{
  uint8_t c;
  static uint8_t packet_type = PACKET_POS;
  static uint8_t config_count = 0;
  static uint8_t init_count = 0;

  if (millis() >= next_tx_millis)
  {
    /*  Reload the timer for the next transmission, do it first so we don't
        add the processing and transmit delays to our interval period */
     next_tx_millis = millis() + APRS_PERIOD;

    // Show modem ISR stats from the previous transmission if enabled
    #ifdef DEBUG_MODEM
      modem_debug();
    #endif

    /*  periodically send the APRS telemetry configuration data, set to approx
        every 40th packet (10mins) */
    if(config_count > APRS_TELEM_CFG_CNT)
    {
      packet_type = PACKET_UNITS;
      config_count = 0;
    }
    else
    {
      config_count += 1;
    }

    /*  Now we can actually get around to sending the packet, so we will send position
        and raw telemetry values on an alternating scheme. Every 40th packet we will
        send the telemetry config packets.  The order in which packets are sent is via
        very simple state machine */
    switch(packet_type)
    {
      case PACKET_POS:  // Send Position Report
             aprs_send();
             packet_type = PACKET_TELEMETRY;
             break;
              
      case PACKET_TELEMETRY:  // Send Telemetry Data
             aprs_telemetry_data();

             /*  start by sending UNIT packets until APRS_TELEM_INIT_CNT exceeded, then
                 go back to sending POS packets and unit packets every 40th */
             if(init_count++ < APRS_TELEM_INIT_CNT)
             {
                packet_type = PACKET_UNITS;
                config_count = 0;
             }
             else
             {
               packet_type = PACKET_POS;
             }
             break;

      case PACKET_UNITS: // Send Telemetry Information
             aprs_telemetry_params();
             packet_type = PACKET_POS;
             break;
    }
  }

       
  if (Serial.available())
  {
    c = Serial.read();
    if (gps_decode(c))
    {
      if (gps_altitude > BUZZER_ALTITUDE)
        buzzer_off();   // In space, no one can hear you buzz
      else
        buzzer_on();
    }
  }
  else
  {
    power_save();
  }
}

