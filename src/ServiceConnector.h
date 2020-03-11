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

// ServiceConnector.h

#ifndef _SERVICECONNECTOR_h
#define _SERVICECONNECTOR_h
#include "logger.h"
#include "application.h"
#include "AFE.h"
#include "VOC.h"
#include "CO2.h"
#include "Sensor.h"
#include "StringStream.h"
#include "PhotonConfig.h"
#include "MQTT.h"
#include "distributions.h"
#include <math.h>

#define READINGS_HISTORY_SIZE 3
#define N_IN 18

typedef struct {
	float CO2;
	float S1A;
	float S1W;
	float S2A;
	float S2W;
	float S3A;
	float S3W;
	float humidity;
	float pressure;
	float temperature;
} Readings_History_t;

class ServiceConnector
{
  public:
    ServiceConnector(Sensor& sensor, VOC& voc, CO2& co2, MQTT& mqtt);
    void begin();
    void processCommands();
    bool updateReadings();
    void processReadings();
    void applyWiFiStatus();

  private:
	Readings_History_t ReadingsHistory[READINGS_HISTORY_SIZE];
	int ReadingsHistoryPos = 0;
	Readings_History_t* UpdateHistory();
	void ComputeStatisticalInputs(float inputs[]);
	float ReadingsHistoryMean(int offset);
	float ReadingsHistoryVar(int offset, float mean);

	float convertRawGasToVoltage(int rng, int rawValue);

    unsigned long startSampleTime;
    unsigned long endSampleTime;

    bool serialBypass;
    //AFE& afe;
    Sensor& sensor;
    VOC& voc;
    CO2& co2;
	MQTT& mqttClient;
};

#endif
