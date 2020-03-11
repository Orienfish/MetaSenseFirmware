// BSD 3-Clause License
//
// Copyright (c) 2018, The Regents of the University of California.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "application.h"

int freeRam();
int processMsg(String extra);
void button_clicked(system_event_t event, int param);
void re_enable_sleep();
void setup();
void loop();
void serialEvent();
void serialEvent1();

SYSTEM_MODE(MANUAL); // to prevent the device from trying to connect 
					 // to cloud before running firmware
SYSTEM_THREAD(ENABLED);
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));
STARTUP(System.enableFeature(FEATURE_RESET_INFO));


#if (PLATFORM_ID == 6)
// Photon code here
PRODUCT_ID(790);
#elif (PLATFORM_ID == 10)
//Electron or other Particle device code here
PRODUCT_ID(2015);
#endif

PRODUCT_VERSION(9);

#include "Adafruit_ADS1015.h"
#include "PhotonConfig.h"
#include "SHT1x.h"
#include "AFE.h"
#include "VOC.h"
#include "CO2.h"
#include "Sensor.h"
#include "ServiceConnector.h"
#include "logger.h"
#include "shape.h"

//Variables retined when in DEEP SLEEP
//retained unsigned long samplingInterval = 5000;
//retained long wifiStatus = -1;
retained bool usbMirror = false;
//retained StreamingType_t streamingType = streamAll;
retained unsigned long lastSetupTime = 0;
retained unsigned long lastReadingTime = 0;
retained unsigned long nextSyncTime = 0;
//retained bool sleepEnabled = true;
//retained bool vocInstalled = false;
//retained bool co2Installed = false;
retained bool init = true;
retained SensorEEPROMConfig_t SensorConfig;

retained adsGain_t currentGain = GAIN_TWOTHIRDS;

retained int BLE_KEY_PIN = D4;
retained int UNCONNECTED_CS_PIN = D6;

// ----------------
bool temporarlyDisableSleep = false;
bool usbPassthrough = false;

STARTUP(WiFi.selectAntenna(ANT_INTERNAL));

//Make sure that the sensor resets if the sensor is stuck in the loop for
//more than a minute
ApplicationWatchdog wd(WATCHDOG_TIMEOUT, System.reset);

int freeRam() {
	uint32_t freemem = System.freeMemory();
	return freemem;
}

Sensor sensor(HumSckPin, HumDataPin, BarCSPin, SDCSPin, UNCONNECTED_CS_PIN, ADS1115_ADDRESS_0, ADS1115_ADDRESS_1);

VOC voc(ADS1115_ADDRESS_0);
CO2 co2;

void mqttCallback(char* topic, byte* payload, unsigned int length);
MQTT mqttClient(MQTT_Server_Address, MQTT_Server_Port, mqttCallback, MAX_MSG_LEN);

ServiceConnector connector(sensor, voc, co2, mqttClient);

char buf[MAX_MSG_LEN+1];
void mqttCallback(char* topic, byte* payload, unsigned int length) {
	INO_TRACE("Processing MQTT message: %s\r\n", topic);
	strncpy(buf, (const char*)payload, MAX_MSG_LEN);
	// TODO add processing functions
	//connector.receiveMessageWiFi(buf);
}

Timer timer(30000, re_enable_sleep, true);
void re_enable_sleep() {
	//Serial1.println("Reenable sleep");
	temporarlyDisableSleep = false;
}

void setup()
{
	// set Sensor Config
	SensorConfig.wifiEnabled = true;
	SensorConfig.sleepEnabled = true;
	SensorConfig.vocInstalled = false;
	SensorConfig.co2Installed = false;
	SensorConfig.intervalTime = INTERVAL_MS;

	if (init)
	{
		usbMirror = false;
		lastSetupTime = 0;
		lastReadingTime = 0;
		nextSyncTime = 0;
		currentGain = GAIN_TWOTHIRDS;
		//ResetSequenceLen = 0;

		if (BOARD_VERSION>=2.2) {
			BLE_KEY_PIN = D4;
			UNCONNECTED_CS_PIN = D6;
		} else {
			BLE_KEY_PIN = D6;
			UNCONNECTED_CS_PIN = D4;
		}
	}

	//Configure seral ports
	Serial.begin(serialSpeed);		//USB uart on photon

	sensor.begin();
	connector.begin();
	if (SensorConfig.vocInstalled)
		voc.begin();
	if (SensorConfig.co2Installed)
		co2.begin();

	init = false;
}

void loop()
{
	digitalWrite(led1, !digitalRead(led1));

	mqttClient.loop();
	if (connector.updateReadings()){
		INO_TRACE("---------Update Readings returned true.---------\n");
		connector.processReadings();
	}
	connector.applyWiFiStatus();
	//Make sure we disable the forced wakeup if the pin goes down
	//sensor.initWakeupPinStatus();
}