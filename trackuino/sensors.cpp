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

/* Credit to:
 *
 * cathedrow for this idea on using the ADC as a volt meter:
 * http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
 */
#include "trackuino.h"
#include "config.h"
#include "sensors.h"
#include <Arduino.h>

#if defined(ONE_WIRE_TEMP)
 #include <OneWire.h>
 #include <DallasTemperature.h>

 // Temp sensor objects. sensor IDs set in config.h
 OneWire oneWire(PIN_ONEWIRE);
 DallasTemperature snsr(&oneWire);

 /* now we need to figure out which sensor is which */
 uint8_t snsr_int[] = INTERNAL_TEMP_ADDR;
 uint8_t snsr_ext[] = EXTERNAL_TEMP_ADDR;

#endif

/*
 * sensors_aref: measure an external voltage hooked up to the AREF pin,
 * optionally (and recommendably) through a pull-up resistor. This is
 * incompatible with all other functions that use internal references
 * (see config.h)
 */
#ifdef USE_AREF
void sensors_setup()
{
  // Nothing to set-up when AREF is in use
}

unsigned long sensors_aref()
{
  unsigned long result;
  // Read 1.1V reference against AREF (p. 262)
  ADMUX = _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = (ADCH << 8) | ADCL;

  // millivolts = 1.1 * 1024 * 1000 / result
  result = 1126400 / result;

  // aref = read aref * (32K + AREF_PULLUP) / 32K
  result = result * (32000UL + AREF_PULLUP) / 32000;

  return result;
}
#endif

#ifndef USE_AREF
void sensors_setup()
{
  #if defined(ONE_WIRE_TEMP)
   /* startup dallas Temperature Sensors */
   snsr.begin();
   snsr.requestTemperatures();
  #endif 
}

#if defined(ONE_WIRE_TEMP)

int sensors_int_ds18b20(void)
{
  /* cast float to int and let compiler deal with it */
  return (int)snsr.getTempC(snsr_int);
}

int sensors_ext_ds18b20(void)
{
  /* cast float to int and let compiler deal with it */
  return (int)snsr.getTempC(snsr_ext);
}

void sensors_request_ds18b20(void)
{
  /* get temperature sensors to perform a conversion */
  snsr.requestTemperatures();
}

#endif

long sensors_internal_temp()
{
  long result;
  // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = (ADCH << 8) | ADCL;
  
  result = (result - 125) * 1075;

  return result;
}

int sensors_lm60(int powerPin, int readPin)
{
  digitalWrite(powerPin, HIGH);   // Turn the LM60 on
  analogReference(INTERNAL);      // Ref=1.1V. Okay up to 108 degC (424 + 6.25*108 = 1100mV)
  analogRead(readPin);            // Disregard the 1st conversion after changing ref (p.256)
  int adc = analogRead(readPin);  // Real read
  digitalWrite(powerPin, LOW);    // Turn the LM60 off
  int mV = 1100L * adc / 1024L;   // Millivolts
  
  switch (TEMP_UNIT)//Added by: Kyle Crockett
  {
	case 1://C
		return (4L * (mV - 424) / 25)+ CALIBRATION_VAL ;    // Vo(mV) = (6.25*T) + 424 -> T = (Vo - 424) * 100 / 625
	break;
	case 2://K
		return (4L * (mV - 424) / 25) + 273 + CALIBRATION_VAL; //C + 273 = K
	break;
	case 3://F
		return (36L * (mV - 424) / 125) + 32+ CALIBRATION_VAL; // (9/5)C + 32 = F
	break;
  };
}

int sensors_ext_lm60()
{
  return sensors_lm60(EXTERNAL_LM60_VS_PIN, EXTERNAL_LM60_VOUT_PIN);
}

int sensors_int_lm60()
{
  return sensors_lm60(INTERNAL_LM60_VS_PIN, INTERNAL_LM60_VOUT_PIN);
}


int sensors_humidity()
{
  // TO DO
  return 0;
}

int sensors_pressure()
{
  // TO DO
  return 0;
}

int sensors_uv_ray()
{
  // Nice to have at 40 km altitude
  return 0;
}

int sensors_gamma_ray()
{
  // http://www.cooking-hacks.com/index.php/documentation/tutorials/geiger-counter-arduino-radiation-sensor-board
  return 0;
}

int sensors_graviton()
{
  // Wait, what?
  return 0;
}

#endif
