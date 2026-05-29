#ifndef SLIMEVR_TELEMETRY_TELEMETRYSERVICE_H_
#define SLIMEVR_TELEMETRY_TELEMETRYSERVICE_H_

#include <Arduino.h>
#include <WiFiUdp.h>

#include "debug.h"
#include "logging/Level.h"

namespace SlimeVR::Telemetry {

class TelemetryService {
public:
	TelemetryService();

	void setup();
	void update();
	void beginLoop();
	void endLoop();

	static void writeLog(
		Logging::Level level,
		const char* prefix,
		const char* tag,
		const char* message
	);

private:
	void sendLine(const char* line);
	void sendPerformance();
	void writeLogLine(
		Logging::Level level,
		const char* prefix,
		const char* tag,
		const char* message
	);

	static TelemetryService* s_Instance;

	WiFiUDP m_UDP;
	IPAddress m_Host = IPAddress(255, 255, 255, 255);
	uint16_t m_Port = TELEMETRY_PORT;
	bool m_Enabled = ENABLE_TELEMETRY;

	uint32_t m_LastPerformanceSend = 0;
	uint32_t m_LoopStartMicros = 0;
	uint32_t m_LoopCount = 0;
	uint64_t m_LoopTotalMicros = 0;
	uint32_t m_LoopMaxMicros = 0;
};

}  // namespace SlimeVR::Telemetry

#endif  // SLIMEVR_TELEMETRY_TELEMETRYSERVICE_H_
