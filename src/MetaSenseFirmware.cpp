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
#include "PowerManager.h"

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

retained PowerEEPROMState_t PowerState;

PowerManager PM;

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

// settings for the adaptive sampling algorithm
int I_max = 40, I_min = 1; // in minutes
float k = 2.12, alpha = 0.4;
retained double last_x = 0.0, cur_x;
retained unsigned long last_time = 0.0, cur_time; // in millis
retained uint16_t last_temp = 0;
int next_interval_s = INTERVAL_S; // in seconds

void setup()
{
	// set Sensor Config
	SensorConfig.wifiEnabled = true;
	SensorConfig.sleepEnabled = true;
	SensorConfig.vocInstalled = false;
	SensorConfig.co2Installed = false;
	SensorConfig.intervalTime = INTERVAL_S;

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

	PM.begin(&PowerState);

	/*if (PM.isBatteryLow() && !PM.isChargingOrTrickling()){
		//TODO for debug remove in production
		PM.printPowerReport();
		//TODO end of debug stuff
		INO_TRACE("---------Low battery in begin. Go down for sleep.---------");
		//Sleep for 60 secs
		System.sleep(SLEEP_MODE_DEEP, 60000);
	}*/

	// If the reset was due to power issues
	if (System.resetReason() == RESET_REASON_POWER_MANAGEMENT ||
					 System.resetReason() == RESET_REASON_POWER_DOWN ||
					 System.resetReason() == RESET_REASON_POWER_BROWNOUT) {
		//TODO: Maybe we need to do something if battery died to reset power module??
		PM.printPowerReport();
		//TODO end of debug stuff
		if (System.resetReason() == RESET_REASON_POWER_MANAGEMENT)
			INO_TRACE("Recovered from reset with reason RESET_REASON_POWER_MANAGEMENT: %d\n", System.resetReason());
		if (System.resetReason() == RESET_REASON_POWER_DOWN)
			INO_TRACE("Recovered from reset with reason RESET_REASON_POWER_DOWN: %d\n", System.resetReason());
		if (System.resetReason() == RESET_REASON_POWER_BROWNOUT)
			INO_TRACE("Recovered from reset with reason RESET_REASON_POWER_BROWNOUT: %d\n", System.resetReason());
		//PM.reset();
	}

	init = false;
	pinMode(DebugLED, OUTPUT);
}

void loop()
{
	digitalWrite(DebugLED, !digitalRead(DebugLED));

	mqttClient.loop();
	if (connector.updateReadings()){
		cur_time = millis(); // get the time for the currret reading
		INO_TRACE("---------Update Readings returned true.---------\n");
		// connector.processReadings();
		AFE::Gas_Model_t* afeModel = &sensor.afe.lastModel;

		// sleep for the interval time
		// implement the adaptive sampling heuristic here
		double delta_h, grad;
		if (last_time != 0.0) {
			delta_h = max(double(cur_time - last_time) / 1000 / 3600, double(I_min) / 60); // in hours
			grad = double(abs(afeModel->temp_F - last_temp)) / delta_h;
			cur_x = alpha * grad + (1 - alpha) * last_x;
			next_interval_s = int(I_max - k * cur_x) * 60; // convert from minutes to seconds
			next_interval_s = max(I_min * 60, next_interval_s); // lower bound in seconds
			INO_TRACE("delta_h: %f, grad: %f, cur_x: %f\n", delta_h, grad, cur_x);
			INO_TRACE("next_interval_s: %d\n", next_interval_s);
		}

		// update last_time, last_temp and last_x
		last_time = cur_time;
		last_temp = afeModel->temp_F;
		last_x = cur_x;

		sprintf(buf, "%.4f,%.4f,%.4f,%.1f,%d,%d,%d,%f,%f,%f,%d\n", afeModel->SN1_ppb, 
			afeModel->SN2_ppm, afeModel->SN3_ppm, afeModel->AQI, 
			afeModel->temp_F, afeModel->hum_RH, PM.getFuelLevel(),
			delta_h, grad, cur_x, next_interval_s);
		connector.publishReadings(buf);

		//PM.updateReadings must be called periodically
		PM.updateReadings();
		PM.printPowerReport();
		if (PM.isBatteryLow() && !PM.isChargingOrTrickling()){
			INO_TRACE("---------Low battery in loop. Go down for sleep.---------\n");
			//Sleep for 60 secs
			System.sleep(SLEEP_MODE_DEEP, 60);
		}
	}
	connector.applyWiFiStatus();
	//Make sure we disable the forced wakeup if the pin goes down
	sensor.initWakeupPinStatus();

	// sleep for the calculated interval
	System.sleep(SLEEP_MODE_DEEP, next_interval_s);
}