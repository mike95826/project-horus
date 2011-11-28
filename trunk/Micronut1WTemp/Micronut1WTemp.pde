/* Micronut 1-Wire Temperature Sensor Address
 * (C) 2011 M. Cook (VK5ZM) & M.Jessop (VK5FDRK)
 *
 * We use this program to print out the addresses of the 
 * DS18S20 temperature sensors we're using.  Once we know
 * the addresses we can feed this back into the trackuino
 * fimrware to keep track of which sensor is the internal
 * and external temp sensor.
 *
 * To get this running you'll need to attach a TTL to
 * serial converter to a Micronut board on pins 5 and 6.
 * Take a look at the NewSoftSerial library to figure out
 * which pin is which, if you get it wrong swap the 
 * definition below *before* you get out your soldering
 * iron.
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
#include <Wire.h>
#include <WProgram.h>

/* We need the following Arduino Libs */
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NewSoftSerial.h>

/* define our constants */
#define PIN_ONEWIRE        A0    // define the pin where the 1-wire bus lives 
#define MAX_DEVICES        16    // maximum number of temperature sensors on the 1-wire bus

/* declare our objects we wish to use */
OneWire oneWire(PIN_ONEWIRE);          // give me a 1-wire bus
DallasTemperature sensor(&oneWire);    // let me talk to temp sensors
NewSoftSerial mySerial(5,6);           //serial is on pins 5 * 6

/* Some variables to hold 1-wire temp sensor addresses */
DeviceAddress tempSensors[MAX_DEVICES];

/* some local variables */
static char txbuffer[100];
static int num_devices;

void setup()
{
    /* get the serial port working, we'll need this */
    mySerial.begin(9600);
    mySerial.print("\r\nBooting up...\r\n");

    /* startup dallas Temperature Sensors */
    sensor.begin();
    sensor.requestTemperatures();
   
    /* find out how many devices there are */
    num_devices = sensor.getDeviceCount();
  
    /* we'll limit ourselves to MAX_SENSORS on the bus */
    if(num_devices > MAX_DEVICES)
      num_devices = MAX_DEVICES;
  
    sprintf(txbuffer,"Found %d sensors...\r\n\r\n",num_devices);
    mySerial.print(txbuffer);

    register unsigned char i;    // need a counter 

    for(i=0; i < num_devices; i++)
    {
        if (!sensor.getAddress(tempSensors[i], i))
        {
            sprintf(txbuffer,  "Unable to find address for Device %d\r\n",i);
            mySerial.print(txbuffer); 
        }
    }
}

void loop()
{
    register unsigned char i;    //need a counter

    /* print the device addresses of each individual temp sensor we found */
    for(i=0; i < num_devices; i++)
    {
        sprintf(txbuffer, "Sensor %d {",i);
        mySerial.print(txbuffer);
        printAddress(tempSensors[i]);
        mySerial.print("}\r\n");
    }
    mySerial.print("\r\nFinished...\r\n");

    while(1){};    // never return
}

void printAddress(DeviceAddress deviceAddress)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        // zero pad the address if necessary
        char temp[5];
        unsigned int tempbyte = 0;
    
        tempbyte = deviceAddress[i];
    
        if (deviceAddress[i] < 16)
          mySerial.print("0");
        sprintf(temp, "%X", tempbyte);
        mySerial.print(temp);
        if (i<7)
          mySerial.print(",");
    }
}
