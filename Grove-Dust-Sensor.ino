/*
 Change History:
 v7.alpha First push from local files into Github.
*/



/*
Some or much of this code is from BME280 I2C Test.ino
This code shows how to record data from the BME280 environmental sensor
using I2C interface. This file is an example file, part of the Arduino
BME280 library.
Copyright (C) 2016  Tyler Glenn

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Written: Dec 30 2015.
Last Updated: Sep 19 2016.

Connecting the BME280 Sensor:
Sensor              ->  Board
-----------------------------
Vin (Voltage In)    ->  3.3V
Gnd (Ground)        ->  Gnd
SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro

*/

/*
 Some or much of this code is from Grove - Dust Sensor Demo v1.0
 Interface to Shinyei Model PPD42NS Particle Sensor
 Program by Christopher Nafis
 Written April 2012

 http://www.seeedstudio.com/depot/grove-dust-sensor-p-1050.html
 http://www.sca-shinyei.com/pdf/PPD42NS.pdf

 JST Pin 1 (Black Wire)  =&gt; //Arduino GND
 JST Pin 3 (Red wire)    =&gt; //Arduino 5VDC
 JST Pin 4 (Yellow wire) =&gt; //WeMOS D3 = GPIO0
*/

/*
 * WiFi Manager: How It Works

    when your ESP starts up, it sets it up in Station mode and tries to connect to a previously saved Access Point
    if this is unsuccessful (or no previous network saved) it moves the ESP into Access Point mode and spins up a 
    DNS and WebServer (default ip 192.168.4.1)
    using any wifi enabled device with a browser (computer, phone, tablet) connect to the newly created Access Point
    because of the Captive Portal and the DNS server you will either get a 'Join to network' type of popup or get 
    any domain you try to access redirected to the configuration portal
    choose one of the access points scanned, enter password, click save
    ESP will try to connect. If successful, it relinquishes control back to your app. If not, reconnect to AP and reconfigure.

 */
/* ==== WiFi Manager Includes ==== */
#include <ESP8266WiFi.h>          // https://github.com/esp8266/Arduino
#include <WiFiUdp.h>              // For NTP service
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

//for LED status
#include <Ticker.h>


/* ==== BME280 Includes ==== */
#include <BME280I2C.h>
#include <Wire.h>                 // Needed for legacy versions of Arduino.

/* ==== ThingSpeak Includes ==== */
#include "ThingSpeak.h"

/* ==== Password Includes ==== */
#include "grove.h"
  // unsigned long myChannelID = xxxxx;
  // const char * myWriteAPIKey = "xxxxx";

/* ====  END Includes ==== */

/* ==== General Defines ==== */
#define SERIAL_BAUD 115200
#define WIFIRESETBUTTON D3
/* ==== END Defines ==== */

/* ==== BME280 Global Variables ==== */
BME280I2C bme;                // Default : forced mode, standby time = 1000 ms
                              // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
bool metric = true;
float temperature(NAN), humidity(NAN), pressure(NAN);

/* ==== Grove Global Variables ==== */
int pin = D5;                              // Need to have a pin WITHOUT a pull-up or pull-down resistor on the input.
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30*1000;     //sample 30s??
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

/* ==== BME280 WiFiManager Global Variables ==== */
Ticker ticker;

/* ==== NTP  Global Variables ==== */
  unsigned int localPort = 2390;            // local port to listen for UDP packets
  
  // Don't hardwire the IP address or we won't get the benefits of the pool.
  // Lookup the IP address for the host name instead
  // IPAddress timeServer(129, 6, 15, 28);  // time.nist.gov NTP server
  IPAddress timeServerIP;                   // time.nist.gov NTP server address
  const char* ntpServerName = "time.nist.gov";
  
  const int NTP_PACKET_SIZE = 48;           // NTP time stamp is in the first 48 bytes of the message
  const long TZOFFSET = 10 * 3600;          // TNS: Rough aproach to Timezone offset in seconds
  
  byte packetBuffer[ NTP_PACKET_SIZE];      //buffer to hold incoming and outgoing packets
  
  // A UDP instance to let us send and receive packets over UDP
  WiFiUDP udp;

/* ====  ThingSpeak Global Variables ==== */
  WiFiClient client;                        // Need this for ThingSpeak?
  //float current_temp;                     // These are used for sending to ThingSpeak
  //float current_humidity;
  

  
  uint32_t delayMS;
  // Set up some time variables
  // 2^32 -1 - give me about 24 days before it rolls over.
  unsigned long oldTime, newTime;
  // 5 minutes = 1000 x 60 x 5 = 300000
  #define THINGSPEAKDELAY 300000
/* ==== END Global Variables ==== */


/* ==== Prototypes ==== */
/* === Print a message to stream with the temp, humidity and pressure. === */
void printBME280Data(Stream * client);
/* === Print a message to stream with the altitude, and dew point. === */
void printBME280CalculatedData(Stream* client);
/* ==== END Prototypes ==== */

/* ==== Setup =========================================================================================== */
void setup() {
  Serial.begin(SERIAL_BAUD);
  while(!Serial) {} // Wait

  // Use the template begin(int SDA, int SCL);
  // On WeMOS Mini D1 we have
  // SDA = D2 = GPIO4
  // SCL = D1 = GPio5
  while(!bme.begin(SDA,SCL)){
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  // Grove just uses a standard IO pin
  pinMode(pin,INPUT);
  // starttime = millis();//get the current time;

  // set a button input to know when to reset the WiFi
  pinMode(WIFIRESETBUTTON, INPUT);
  
  // set led pin as output
  pinMode(BUILTIN_LED, OUTPUT);

  // This loops until the WIFI is configured
  configureWIFI(false);

  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());

  // Instantiate ThingSpeak
  ThingSpeak.begin(client);

  oldTime = millis();                 // Note when we started
 
}
/* ==== END Setup ==== */

/* ==== Loop =============================================================================================== */
void loop() {
  // Measure the dust using the Grove sensor, noting this is a 30 second looping code notionally
  // We want to only measure every 5 minutes
  newTime = millis();

  if((newTime - oldTime) > THINGSPEAKDELAY){
    oldTime = newTime;
    doDustMeasurement();
    
    getNTPTime();
    
    // BME code, and we'll have a nap at the end
    takeBME280Reading();
    printBME280Data(&Serial);
    printBME280CalculatedData(&Serial);
    // Write it to the CLoud
    writeThingSpeak();

  }

  // Check to see if the WiFi reset button is pressed (LOW)
  // I'm sure this can be done neater.
  if (!digitalRead(WIFIRESETBUTTON)){
    // OK the reset button is set
    Serial.println("WIFIRESETBUTTON pressed");
    configureWIFI(true);
  }
}
/* ==== End Loop ==== */







/* ==== Functions ==== */
// ThingSpeak uploader routine
void writeThingSpeak(){
  // LED ON to indicate we are handling a request
  digitalWrite(BUILTIN_LED, LOW);  
  // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
  // pieces of information in a channel.  
  // Set the fields to be set in ThingSpeak
  ThingSpeak.setField(1,temperature);
  ThingSpeak.setField(2,humidity);
  ThingSpeak.setField(3,concentration);
  ThingSpeak.setField(4,static_cast<long>(lowpulseoccupancy));

  // Write the fields to ThingSpeak
  ThingSpeak.writeFields(myChannelID, myWriteAPIKey);

  Serial.print("Sent to ThingSpeak:: ");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("  Humidity: ");
  Serial.println(humidity);
  Serial.print("  Dust: ");
  Serial.println(concentration);
  
  // LED OFF to indicate we are finished
  digitalWrite(BUILTIN_LED, HIGH);    
}



void configureWIFI(boolean ClearWIFI){
  // Generalised function to reset the WiFi if needed
  // ClearWIFI = true will force clearing of the WIFI settings
    
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);  

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around, hence local.
  WiFiManager wifiManager;

  if (ClearWIFI){
    // reset settings
    Serial.println("Clearing WIFI settings");
    wifiManager.resetSettings();
    ESP.reset();
  }


  // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  // fetches ssid and pass and tries to connect
  // if it does not connect it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    // reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  } 

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  ticker.detach();
  //keep LED on
  digitalWrite(BUILTIN_LED, LOW);
}


void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

void tick(){
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}


void doDustMeasurement(void){
  /*
   * The Grove sensor runs in a tight loop, so it will need to consider impacts if there is any other looping code.
   *
   *
   * Reads a pulse (either HIGH or LOW) on a pin. For example, if value is HIGH, pulseIn() 
   * waits for the pin to go HIGH, starts timing, then waits for the pin to go LOW and 
   * stops timing. Returns the length of the pulse in microseconds or 0 if no complete 
   * pulse was received within the timeout.
   */
  lowpulseoccupancy = 0;  
  starttime = millis();                                               //get the current time; 
  // Loop for sampltime_ms duration reading the pulse measurements and accumulating them
  while ((millis()-starttime) < sampletime_ms){
    duration = pulseIn(pin, LOW);                                     // Timeout is 1 second as default when not specified.
    lowpulseoccupancy = lowpulseoccupancy+duration;                   // Duration is in microseconds
  }


  /*
   * This could do with some tweaking as the sample period could extend if pulseIn() forced the 
   * sample time to extend beyond 30 secs.  Perhaps setting a timeout is appropriate?
   */
  ratio = lowpulseoccupancy/(sampletime_ms*10.0);                     // Integer percentage 0 < 100
  concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62;   // using spec sheet curve
  Serial.print("lowpulseoccupancy = ");
  Serial.print(lowpulseoccupancy);
  Serial.println(" microseconds");
  Serial.println("\n");    
  Serial.print("concentration = ");
  Serial.print(concentration);
  Serial.println(" pcs/0.01cf");
  Serial.println("\n");

}

void takeBME280Reading(void){
  // TNS - Notionally we are in "forced mode" which means we need to trigger the read then wait 8ms
  bme.setMode(0x01);
  delay(10);
 
  uint8_t pressureUnit(3);                                           // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
  bme.read(pressure, temperature, humidity, metric, pressureUnit);                   // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
  /* Alternatives to ReadData():
  float temp(bool celsius = false);
  float pres(uint8_t unit = 0);
  float hum();
  */  
  
}



void printBME280Data(Stream* client){
  /*
  Keep in mind the temperature is used for humidity and
  pressure calculations. So it is more effcient to read
  temperature, humidity and pressure all together.
  */
  client->print("Temp: ");
  client->print(temperature);
  client->print("°"+ String(metric ? 'C' :'F'));
  client->print("\t\tHumidity: ");
  client->print(humidity);
  client->print("% RH");
  client->print("\t\tPressure: ");
  client->print(pressure);
  client->print(" atm");
}


void printBME280CalculatedData(Stream* client){
  float altitude = bme.alt(metric);
  float dewPoint = bme.dew(metric);
  client->print("\t\tAltitude: ");
  client->print(altitude);
  client->print((metric ? "m" : "ft"));
  client->print("\t\tDew point: ");
  client->print(dewPoint);
  client->println("°"+ String(metric ? 'C' :'F'));

}

void getNTPTime()
{
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP); 

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  
  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
  }
  else {
    Serial.print("packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = " );
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears + TZOFFSET;
    // print Unix time:
    Serial.println(epoch);


    // print the hour, minute and second:
    Serial.print("The local time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if ( ((epoch % 3600) / 60) < 10 ) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ( (epoch % 60) < 10 ) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
    // TNS: Using casts relatively safely given the calculation being asked for and the 
    // expected return values.
    //hours = static_cast<int>((epoch  % 86400L) / 3600);
    //minutes = static_cast<int>((epoch  % 3600) / 60);
    //seconds = static_cast<int>(epoch % 60);
  }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

/* ==== END Functions ==== */
