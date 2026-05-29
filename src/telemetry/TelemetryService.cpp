#include "TelemetryService.h"

#include "GlobalVars.h"
#include "debug.h"
#include "logging/Level.h"

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>
#endif

namespace SlimeVR::Telemetry {

TelemetryService* TelemetryService::s_Instance = nullptr;

TelemetryService::TelemetryService() { s_Instance = this; }

void TelemetryService::setup() {
#ifdef TELEMETRY_HOST
	m_Host.fromString(TELEMETRY_HOST);
#endif
}

void TelemetryService::beginLoop() { m_LoopStartMicros = micros(); }

void TelemetryService::endLoop() {
	if (!m_Enabled || m_LoopStartMicros == 0) {
		return;
	}

	uint32_t elapsedMicros = micros() - m_LoopStartMicros;
	m_LoopStartMicros = 0;
	m_LoopCount++;
	m_LoopTotalMicros += elapsedMicros;
	if (elapsedMicros > m_LoopMaxMicros) {
		m_LoopMaxMicros = elapsedMicros;
	}
}

void TelemetryService::update() {
	if (!m_Enabled || WiFi.status() != WL_CONNECTED) {
		return;
	}

	uint32_t now = millis();
	if (now - m_LastPerformanceSend >= TELEMETRY_INTERVAL_MS) {
		m_LastPerformanceSend = now;
		sendPerformance();
	}
}

void TelemetryService::writeLog(
	Logging::Level level,
	const char* prefix,
	const char* tag,
	const char* message
) {
	if (s_Instance == nullptr) {
		return;
	}

	s_Instance->writeLogLine(level, prefix, tag, message);
}

void TelemetryService::writeLogLine(
	Logging::Level level,
	const char* prefix,
	const char* tag,
	const char* message
) {
	if (!m_Enabled || WiFi.status() != WL_CONNECTED) {
		return;
	}

	char line[300];
	if (tag == nullptr) {
		snprintf(
			line,
			sizeof(line),
			"log ts=%lu level=%s source=%s msg=%s",
			millis(),
			Logging::levelToString(level),
			prefix,
			message
		);
	} else {
		snprintf(
			line,
			sizeof(line),
			"log ts=%lu level=%s source=%s:%s msg=%s",
			millis(),
			Logging::levelToString(level),
			prefix,
			tag,
			message
		);
	}

	sendLine(line);
}

void TelemetryService::sendPerformance() {
	uint32_t loopAvgMicros = 0;
	if (m_LoopCount > 0) {
		loopAvgMicros = m_LoopTotalMicros / m_LoopCount;
	}

	char line[256];
	snprintf(
		line,
		sizeof(line),
		"perf ts=%lu loop_count=%lu loop_avg_us=%lu loop_max_us=%lu heap=%lu "
		"rssi=%d battery_v=%.3f battery_pct=%.3f",
		millis(),
		static_cast<unsigned long>(m_LoopCount),
		static_cast<unsigned long>(loopAvgMicros),
		static_cast<unsigned long>(m_LoopMaxMicros),
		static_cast<unsigned long>(ESP.getFreeHeap()),
		WiFi.RSSI(),
		battery.getVoltage(),
		battery.getLevel()
	);

	m_LoopCount = 0;
	m_LoopTotalMicros = 0;
	m_LoopMaxMicros = 0;

	sendLine(line);
}

void TelemetryService::sendLine(const char* line) {
	if (m_UDP.beginPacket(m_Host, m_Port) == 0) {
		return;
	}

	m_UDP.write(reinterpret_cast<const uint8_t*>(line), strlen(line));
	m_UDP.endPacket();
}

}  // namespace SlimeVR::Telemetry
