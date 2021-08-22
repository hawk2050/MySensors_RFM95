/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - tekka
 *
 * DESCRIPTION
 * ATC mode settings and signal report functions, on RFM69 and RFM95 nodes
 *
 */


// Enable debug prints
#define MY_DEBUG

// Enable signal report functionalities
#define MY_SIGNAL_REPORT_ENABLED
#define MY_GPS_REPORT_ENABLED


// Enable and select radio type attached

// RFM69
//#define MY_RADIO_RFM69
//#define MY_RFM69_NEW_DRIVER   // ATC on RFM69 works only with the new driver (not compatible with old=default driver)
//#define MY_RFM69_ATC_TARGET_RSSI_DBM (-70)  // target RSSI -70dBm
//#define MY_RFM69_MAX_POWER_LEVEL_DBM (10)   // max. TX power 10dBm = 10mW

// RFM95
#define MY_RADIO_RFM95
#define MY_RFM95_ATC_TARGET_RSSI_DBM (-70)  // target RSSI -70dBm
#define MY_RFM95_MAX_POWER_LEVEL_DBM (10)   // max. TX power 10dBm = 10mW


#define MY_NODE_ID 24
/*Makes this static so won't try and find another parent if communication with
gateway fails*/
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC

/**
 * @def MY_TRANSPORT_WAIT_READY_MS
 * @brief Timeout in ms until transport is ready during startup, set to 0 for no timeout
 */
#define MY_TRANSPORT_WAIT_READY_MS (1000)

// GPS position send interval (in millisectonds)
#define GPS_SEND_INTERVAL 10000

#ifdef MY_GPS_REPORT_ENABLED
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

void updateTime();
#endif

void gps_loop();

#include <MySensors.h>

// ID of the sensor child
#define CHILD_ID_UPLINK_QUALITY (0)
#define CHILD_ID_TX_LEVEL       (1)
#define CHILD_ID_TX_PERCENT     (2)
#define CHILD_ID_TX_RSSI        (3)
#define CHILD_ID_RX_RSSI        (4)
#define CHILD_ID_TX_SNR         (5)
#define CHILD_ID_RX_SNR         (6)
#define CHILD_ID_GPS			(7)

// Initialize general message
MyMessage msgTxRSSI(CHILD_ID_TX_RSSI, V_CUSTOM);
MyMessage msgRxRSSI(CHILD_ID_RX_RSSI, V_CUSTOM);
MyMessage msgTxSNR(CHILD_ID_TX_SNR, V_CUSTOM);
MyMessage msgRxSNR(CHILD_ID_RX_SNR, V_CUSTOM);
MyMessage msgTxLevel(CHILD_ID_TX_LEVEL, V_CUSTOM);
MyMessage msgTxPercent(CHILD_ID_TX_PERCENT, V_CUSTOM);
MyMessage msgUplinkQuality(CHILD_ID_UPLINK_QUALITY, V_CUSTOM);

#ifdef MY_GPS_REPORT_ENABLED
MyMessage msg(CHILD_ID_GPS, V_POSITION);

// This is where the pin TX pin of your GPS sensor is connected to the arduino
static const int GPS_PIN = A0;

// GPS Baud rate (note this is not the same as your serial montor baudrate). Most GPS modules uses 9600 bps.
static const uint32_t GPSBaud = 9600;

// Offset hours adjustment from gps time (UTC)
const int offset = 1;

// TinyGPS++ is used for parsing serial gps data
TinyGPSPlus gps;

// The serial connection to the GPS device
// A5 pin can be left unconnected
SoftwareSerial ss(GPS_PIN, A5);

// Last time GPS position was sent to controller
unsigned long lastGPSSent = 0;

// Some buffers
char latBuf[11];
char lngBuf[11];
char altBuf[6];
char payload[30];
char sz[64];

#endif

void setup()
{
	Serial.begin(115200);

	// Set baudrate form gps communication
	#ifdef MY_GPS_REPORT_ENABLED
	ss.begin(GPSBaud);
	#endif
}


void presentation()
{
	// Send the sketch version information to the gateway and controller
	
#ifdef MY_GPS_REPORT_ENABLED
	sendSketchInfo("GPS Sensor", "1.0");
#else
	sendSketchInfo("ATC", "1.1");
#endif

	// Register all sensors to gw (they will be created as child devices)
	present(CHILD_ID_UPLINK_QUALITY, S_CUSTOM, "UPLINK QUALITY RSSI");
	present(CHILD_ID_TX_LEVEL, S_CUSTOM, "TX LEVEL DBM");
	present(CHILD_ID_TX_PERCENT, S_CUSTOM, "TX LEVEL PERCENT");
	present(CHILD_ID_TX_RSSI, S_CUSTOM, "TX RSSI");
	present(CHILD_ID_RX_RSSI, S_CUSTOM, "RX RSSI");
	present(CHILD_ID_TX_SNR, S_CUSTOM, "TX SNR");
	present(CHILD_ID_RX_SNR, S_CUSTOM, "RX SNR");
#ifdef MY_GPS_REPORT_ENABLED
	present(CHILD_ID_GPS, S_GPS, "GPS INFO");
#endif
}

void loop()
{
	// send messages to GW
	send(msgUplinkQuality.set(transportGetSignalReport(SR_UPLINK_QUALITY)));
	send(msgTxLevel.set(transportGetSignalReport(SR_TX_POWER_LEVEL)));
	send(msgTxPercent.set(transportGetSignalReport(SR_TX_POWER_PERCENT)));
	// retrieve RSSI / SNR reports from incoming ACK
	send(msgTxRSSI.set(transportGetSignalReport(SR_TX_RSSI)));
	send(msgRxRSSI.set(transportGetSignalReport(SR_RX_RSSI)));
	send(msgTxSNR.set(transportGetSignalReport(SR_TX_SNR)));
	send(msgRxSNR.set(transportGetSignalReport(SR_RX_SNR)));

	gps_loop();
	#ifdef MY_GPS_REPORT_ENABLED
	wait(GPS_SEND_INTERVAL);
	#else
	// wait a bit
	wait(5000);
	#endif
}


void gps_loop()
{
#ifdef MY_GPS_REPORT_ENABLED
  unsigned long currentTime = millis();

  // Evaluate if it is time to send a new position
  bool timeToSend = currentTime - lastGPSSent > GPS_SEND_INTERVAL;

  // Read gps data
  while (ss.available())
    gps.encode(ss.read());

  if (timeToSend) {
    // Sync gps time with Arduino
    updateTime();

    // Send current gps location
    if (gps.location.isValid() && gps.altitude.isValid()) {
      // Build position and altitude string to send
      dtostrf(gps.location.lat(), 1, 6, latBuf);
      dtostrf(gps.location.lng(), 1, 6, lngBuf);
      dtostrf(gps.altitude.meters(), 1, 0, altBuf);
      sprintf(payload, "%s;%s;%s", latBuf, lngBuf, altBuf);

      Serial.print(F("Position: "));
      Serial.println(payload);

      send(msg.set(payload));

      Serial.print(F("GPS Time: "));
      sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d", month(), day(), year(), hour(), minute(), second());
      Serial.println(sz);


    } else {
      if (millis() > 5000 && gps.charsProcessed() < 10)
        Serial.println(F("No GPS data received: check wiring"));
      else
        Serial.println(F("No GPS data received yet..."));
    }
    lastGPSSent = currentTime;
  }
#endif
}

#ifdef MY_GPS_REPORT_ENABLED
void updateTime()
{
  TinyGPSDate d = gps.date;
  TinyGPSTime t = gps.time;
  if (d.isValid() && t.isValid()) {
    // set the Time to the latest GPS reading if less then 0.2 seconds old
    setTime(t.hour(), t.minute(), t.second(), d.day(), d.month(), d.year());
    adjustTime(offset * SECS_PER_HOUR);
    return;
  }
  Serial.println(F("Unable to adjust time from GPS"));
}
#endif