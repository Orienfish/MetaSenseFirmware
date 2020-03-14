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

#include "ServiceConnector.h"
SdFat SD;

ServiceConnector::ServiceConnector(Sensor& _sensor,
	VOC& _voc, CO2& _co2, MQTT& mqtt) :
	sensor(_sensor), voc(_voc), co2(_co2), mqttClient(mqtt)
{
//
}

void ServiceConnector::begin()
{
	WiFi.on();
	WiFi.setCredentials(wifiSSID, wifiPassword);
	WiFi.connect(WIFI_CONNECT_SKIP_LISTEN);
	// block until connect to local router
	for (; !WiFi.localIP(); Particle.process());
}

void ServiceConnector::applyWiFiStatus() {
	if (!WiFi.connecting() && !WiFi.ready()) {
		SC_TRACE("Enabling wifi\r\n");
		WiFi.on();
		WiFi.connect(WIFI_CONNECT_SKIP_LISTEN);
		// block until connect to local router
		for (; !WiFi.localIP(); Particle.process());
	}
}

bool ServiceConnector::updateReadings()
{
	sensor.checkWakeupPinStatus();

	unsigned long nowTime = millis();
	unsigned long waited = nowTime - lastReadingTime;
	if (waited>=0 && waited<=SensorConfig.intervalTime)
		System.sleep(SLEEP_MODE_DEEP, waited / 1000);
		// return false;

	//Reinitialize the reading time for a new sample
	//We use second precision that is supported by the real time clock
	lastReadingTime = nowTime; //original was using millis();
	//Read all raw values
	sensor.readAll(); 	// sensor.lastReading;
	sensor.afe.convertModel(sensor.lastReading.hum_T / 100.0, sensor.lastReading.hum_H / 100.0, sensor.lastReading.bar_P / 10.0);
	if (SensorConfig.vocInstalled) {
		sensor.checkWakeupPinStatus();
		voc.readAll(); 		// voc.lastReading;
		voc.convertModel(sensor.lastReading.hum_T / 100.0, sensor.lastReading.hum_H / 100.0, sensor.lastReading.bar_P / 10.0);
	}
	if (SensorConfig.co2Installed) {
		sensor.checkWakeupPinStatus();
		co2.readAll();
		co2.convertModel(sensor.lastReading.hum_T / 100.0, sensor.lastReading.hum_H / 100.0, sensor.lastReading.bar_P / 10.0);
	}
	sensor.checkWakeupPinStatus();
	return true;
}

float ServiceConnector::convertRawGasToVoltage(int rng, int rawValue) {
  	float gain = rng;
  	if (rng == 0)
  		gain = 2.0 / 3.0;
  	float voltCalc = 4096.0 / (gain * 0x7FFF);
  	return (rawValue * voltCalc);
}

Readings_History_t* ServiceConnector::UpdateHistory() {
	if (ReadingsHistoryPos>=READINGS_HISTORY_SIZE){
		ReadingsHistoryPos=0;
	}
	return &ReadingsHistory[ReadingsHistoryPos++];
}
float ServiceConnector::ReadingsHistoryMean(int offset) {
	float acc = 0;
	for (int i=0; i<READINGS_HISTORY_SIZE; i++)
		acc+=*(float*)(&ReadingsHistory[i]+offset);
	return acc / READINGS_HISTORY_SIZE;
}
float ServiceConnector::ReadingsHistoryVar(int offset, float mean){
	float acc = 0;
	for (int i=0; i<READINGS_HISTORY_SIZE; i++)
		acc+=pow(*(float*)(&ReadingsHistory[i]+offset)-mean, 2);
	return acc / (READINGS_HISTORY_SIZE-1);
}

void ServiceConnector::ComputeStatisticalInputs(float inputs[]) {
	// compute statistical value based on historical inputs
	inputs[0] = ReadingsHistoryMean(offsetof(Readings_History_t, CO2));
	inputs[1] = ReadingsHistoryVar(offsetof(Readings_History_t, CO2),inputs[0]);
	inputs[2] = ReadingsHistoryMean(offsetof(Readings_History_t, S1A));
	inputs[3] = ReadingsHistoryVar(offsetof(Readings_History_t, S1A),inputs[2]);
	inputs[4] = ReadingsHistoryMean(offsetof(Readings_History_t, S1W));
	inputs[5] = ReadingsHistoryVar(offsetof(Readings_History_t, S1W),inputs[4]);
	inputs[6] = ReadingsHistoryMean(offsetof(Readings_History_t, S2A));
	inputs[7] = ReadingsHistoryVar(offsetof(Readings_History_t, S2A),inputs[6]);
	inputs[8] = ReadingsHistoryMean(offsetof(Readings_History_t, S2W));
	inputs[9] = ReadingsHistoryVar(offsetof(Readings_History_t, S2W),inputs[8]);
	inputs[10] = ReadingsHistoryMean(offsetof(Readings_History_t, S3A));
	inputs[11] = ReadingsHistoryVar(offsetof(Readings_History_t, S3A),inputs[10]);
	inputs[12] = ReadingsHistoryMean(offsetof(Readings_History_t, S3W));
	inputs[13] = ReadingsHistoryVar(offsetof(Readings_History_t, S3W),inputs[12]);
	inputs[14] = ReadingsHistoryMean(offsetof(Readings_History_t, humidity));
	inputs[15] = ReadingsHistoryVar(offsetof(Readings_History_t, humidity),inputs[14]);
	inputs[16] = ReadingsHistoryMean(offsetof(Readings_History_t, temperature));
	inputs[17] = ReadingsHistoryVar(offsetof(Readings_History_t, temperature),inputs[16]);

	inputs[0] = (inputs[0] - metasense_CO2_mean_mean)/metasense_CO2_mean_variance;
	inputs[1] = (inputs[1] - metasense_CO2_var_mean)/metasense_CO2_var_variance;
	inputs[2] = (inputs[2] - metasense_S1A_mean_mean)/metasense_S1A_mean_variance;
	inputs[3] = (inputs[3] - metasense_S1A_var_mean)/metasense_S1A_var_variance;
	inputs[4] = (inputs[4] - metasense_S1W_mean_mean)/metasense_S1W_mean_variance;
	inputs[5] = (inputs[5] - metasense_S1W_var_mean)/metasense_S1W_var_variance;
	inputs[6] = (inputs[6] - metasense_S2A_mean_mean)/metasense_S2A_mean_variance;
	inputs[7] = (inputs[7] - metasense_S2A_var_mean)/metasense_S2A_var_variance;
	inputs[8] = (inputs[8] - metasense_S2W_mean_mean)/metasense_S2W_mean_variance;
	inputs[9] = (inputs[9] - metasense_S2W_var_mean)/metasense_S2W_var_variance;
	inputs[10] = (inputs[10] - metasense_S3A_mean_mean)/metasense_S3A_mean_variance;
	inputs[11] = (inputs[11] - metasense_S3A_var_mean)/metasense_S3A_var_variance;
	inputs[12] = (inputs[12] - metasense_S3W_mean_mean)/metasense_S3W_mean_variance;
	inputs[13] = (inputs[13] - metasense_S3W_var_mean)/metasense_S3W_var_variance;
	inputs[14] = (inputs[14] - metasense_humidity_mean_mean)/metasense_humidity_mean_variance;
	inputs[15] = (inputs[15] - metasense_humidity_var_mean)/metasense_humidity_var_variance;
	inputs[16] = (inputs[16] - metasense_temperature_mean_mean)/metasense_temperature_mean_variance;
	inputs[17] = (inputs[17] - metasense_temperature_var_mean)/metasense_temperature_var_variance;
}

void ServiceConnector::processReadings() {
	//Convert and send as needed by the streaming mode
	// uint32_t timestamp = Time.now();
	//Get the history buffer (it's circular right now size 3)
	Readings_History_t* currentHistory = UpdateHistory();
	//Update the values of the buffer with current readings
	currentHistory-> CO2 = co2.lastReading.val;
	currentHistory-> S1A = convertRawGasToVoltage(sensor.afe.lastReading.range, sensor.afe.lastReading.ADC0_0);
	currentHistory-> S1W = convertRawGasToVoltage(sensor.afe.lastReading.range, sensor.afe.lastReading.ADC0_1);
	currentHistory-> S2A = convertRawGasToVoltage(sensor.afe.lastReading.range, sensor.afe.lastReading.ADC1_2);
	currentHistory-> S2W = convertRawGasToVoltage(sensor.afe.lastReading.range, sensor.afe.lastReading.ADC1_3);
	currentHistory-> S3A = convertRawGasToVoltage(sensor.afe.lastReading.range, sensor.afe.lastReading.ADC1_0);
	currentHistory-> S3W = convertRawGasToVoltage(sensor.afe.lastReading.range, sensor.afe.lastReading.ADC1_1);
	currentHistory-> humidity=sensor.lastReading.hum_H / 100.0;
	currentHistory-> pressure=sensor.lastReading.bar_P / 10.0;
	currentHistory-> temperature=sensor.lastReading.bar_T  / 10.0;

	// process the history buffer (it's currently size 3)
	// smooth the input
	float inputs[N_IN];
	ComputeStatisticalInputs(inputs);


	// publish msg using MQTT
	//M_MQTT_TRACE("Publishing MSG to MQTT: %s\r\n", msg);
	//if (!mqttClient.isConnected()) {
	//	M_MQTT_TRACE("Not connected to MQTT... reconnecting\r\n");
		//mqttClient.connect(MQTT_Client_ID, MQTT_Server_Username, MQTT_Server_Password);
	//	mqttClient.connect(MQTT_Client_ID);
	//}
	//if (mqttClient.isConnected()) {
	//	M_MQTT_TRACE("Client is connected... publish to %s\r\n", MQTT_Topic);
	//	mqttClient.publish(MQTT_Topic, msg);
	//}
}
