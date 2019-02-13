Grove-Dust-Sensor
=================

Use a **Grove Dust Sensor** to send information to ThingSpeak as an IOT experiment.

The Grove Dust Sensor is a nifty little device to see how dirty my workshop air is (I do woodworking hence it is pretty bad).
Reference:  http://wiki.seeedstudio.com/Grove-Dust_Sensor/

In addition a BME280 I2C based temperature and humidity sensor is included.
* Library=BME280
* Version=2.1.4  **NOTE: This version of the code will NOT compile with v2.2 or higher.**
* Author=Tyler Glenn <protglenn@gmail.com*
* The library provides a library for reading and interpreting Bosch BME280 environmental sensor data over I2C and SPI.
* paragraph=Reads temperature, humidity, and pressure. Calculates altitude and dew point. Provides functions for english and metric. 
* Also reads pressure in Pa, hPa, inHg, atm, bar, torr, N/m^2 and psi.

This has all been paired with an **ESP8266 module** to send data to **ThingsSpeak**.

**WiFi Manager** is used for pairing to my home WiFi when your ESP starts up, it sets it up in Station mode and tries to connect
to a previously saved Access Point. If this is unsuccessful (or no previous network saved) it moves the ESP into Access Point
mode and spins up a DNS and WebServer (default ip 192.168.4.1).
Using any wifi enabled device with a browser (computer, phone, tablet) connect to the newly created Access Point
because of the Captive Portal and the DNS server you will either get a 'Join to network' type of popup or get 
any domain you try to access redirected to the configuration portal choose one of the access points scanned, enter password, 
click save and the ESP will try to connect. If successful, it relinquishes control back to your app. 
If not, reconnect to AP and reconfigure.
* Library=WiFiManager
* Version=0.12	Not verified with other versions
* Author=tzapu
* ESP8266 WiFi Connection manager with fallback web configuration portal
* Library for configuring ESP8266 modules WiFi credentials at runtime.
