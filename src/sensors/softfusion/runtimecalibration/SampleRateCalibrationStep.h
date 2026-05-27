/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#pragma once

#include <cmath>
#include <optional>

#include "CalibrationStep.h"

namespace SlimeVR::Sensors::RuntimeCalibration {

template <typename SensorRawT>
class SampleRateCalibrationStep : public CalibrationStep<SensorRawT> {
	using CalibrationStep<SensorRawT>::sensorConfig;
	using typename CalibrationStep<SensorRawT>::TickResult;

public:
	SampleRateCalibrationStep(
		SlimeVR::Configuration::RuntimeCalibrationSensorConfig& sensorConfig
	)
		: CalibrationStep<SensorRawT>{sensorConfig} {}

	void start() override final {
		CalibrationStep<SensorRawT>::start();
		calibrationData = {millis()};
	}

	TickResult tick() override final {
		float elapsedTime = (millis() - calibrationData.value().startMillis) / 1e3f;

		if (elapsedTime < samplingRateCalibrationSeconds) {
			return TickResult::CONTINUE;
		}

		const auto& data = calibrationData.value();
		if (data.accelSamples == 0 || data.gyroSamples == 0 || data.tempSamples == 0) {
			return TickResult::SKIP;
		}

		float accelTimestep = elapsedTime / data.accelSamples;
		float gyroTimestep = elapsedTime / data.gyroSamples;
		float tempTimestep = elapsedTime / data.tempSamples;

		if (sensorConfig.sensorTimestepsCalibrated
			&& !isSignificantChange(sensorConfig.A_Ts, accelTimestep)
			&& !isSignificantChange(sensorConfig.G_Ts, gyroTimestep)
			&& !isSignificantChange(sensorConfig.T_Ts, tempTimestep)) {
			return TickResult::SKIP;
		}

		sensorConfig.A_Ts = accelTimestep;
		sensorConfig.G_Ts = gyroTimestep;
		sensorConfig.T_Ts = tempTimestep;
		sensorConfig.sensorTimestepsCalibrated = true;

		return TickResult::DONE;
	}

	void cancel() override final { calibrationData.reset(); }
	bool requiresRest() override final { return false; }

	void signalOverwhelmed() override final {
		// Restart calibration after an overrun.
		calibrationData.value().accelSamples = 0;
		calibrationData.value().gyroSamples = 0;
		calibrationData.value().tempSamples = 0;
		calibrationData.value().startMillis = millis();
	}

	void processAccelSample(const SensorRawT accelSample[3]) override final {
		calibrationData.value().accelSamples++;
	}

	void processGyroSample(const SensorRawT gyroSample[3]) override final {
		calibrationData.value().gyroSamples++;
	}

	void processTempSample(float tempSample) override final {
		calibrationData.value().tempSamples++;
	}

private:
	static constexpr float samplingRateCalibrationSeconds = 5;
	static constexpr float timestepChangeThreshold = 0.01f;

	static bool isSignificantChange(float oldTimestep, float newTimestep) {
		float baseline = std::abs(oldTimestep);
		if (baseline <= 0.0f) {
			return true;
		}
		return std::abs(oldTimestep - newTimestep) / baseline > timestepChangeThreshold;
	}

	struct CalibrationData {
		uint64_t startMillis = 0;
		uint64_t accelSamples = 0;
		uint64_t gyroSamples = 0;
		uint64_t tempSamples = 0;
	};

	std::optional<CalibrationData> calibrationData;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
