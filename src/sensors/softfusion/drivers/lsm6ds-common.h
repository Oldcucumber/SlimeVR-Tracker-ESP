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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "../../../sensorinterface/RegisterInterface.h"
#include "callbacks.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

struct LSM6DSOutputHandler {
	LSM6DSOutputHandler(
		RegisterInterface& registerInterface,
		SlimeVR::Logging::Logger& logger
	)
		: m_RegisterInterface(registerInterface)
		, m_Logger(logger) {}

	RegisterInterface& m_RegisterInterface;
	SlimeVR::Logging::Logger& m_Logger;

#pragma pack(push, 1)
	struct FifoEntryAligned {
		union {
			int16_t xyz[3];
			uint8_t raw[6];
		};
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 1;

	static float halfToFloat(uint16_t half) {
		const uint32_t sign = static_cast<uint32_t>(half & 0x8000U) << 16U;
		const uint32_t exponent = (half >> 10U) & 0x1FU;
		const uint32_t fraction = half & 0x03FFU;

		uint32_t bits;
		if (exponent == 0U) {
			if (fraction == 0U) {
				bits = sign;
			} else {
				uint32_t normalizedFraction = fraction << 1U;
				uint32_t normalizedExponent = 0U;
				while ((normalizedFraction & 0x0400U) == 0U) {
					normalizedFraction <<= 1U;
					normalizedExponent++;
				}
				normalizedFraction &= 0x03FFU;
				bits = sign
					 | ((127U - 15U - normalizedExponent) << 23U)
					 | (normalizedFraction << 13U);
			}
		} else if (exponent == 0x1FU) {
			bits = sign | 0x7F800000U | (fraction << 13U);
		} else {
			bits = sign | ((exponent + (127U - 15U)) << 23U) | (fraction << 13U);
		}

		float out;
		std::memcpy(&out, &bits, sizeof(out));
		return out;
	}

	static uint16_t readLe16(const uint8_t* data) {
		return static_cast<uint16_t>(data[0])
			 | (static_cast<uint16_t>(data[1]) << 8U);
	}

	static SflpGameRotationVector decodeSflpGameRotation(const FifoEntryAligned& entry) {
		const float x = halfToFloat(readLe16(&entry.raw[0]));
		const float y = halfToFloat(readLe16(&entry.raw[2]));
		const float z = halfToFloat(readLe16(&entry.raw[4]));
		const float ww = std::max(0.0f, 1.0f - x * x - y * y - z * z);

		return SflpGameRotationVector{
			.x = x,
			.y = y,
			.z = z,
			.w = std::sqrt(ww),
		};
	}

	template <typename Regs, size_t MaxReadings = 8>
	bool bulkRead(
		DriverCallbacks<int16_t>&& callbacks,
		float GyrTs,
		float AccTs,
		float TempTs,
		float SflpTs = 0.0f
	) {
		constexpr auto FIFO_SAMPLES_MASK = 0x3ff;
		constexpr auto FIFO_OVERRUN_LATCHED_MASK = 0x800;

		const auto fifo_status = m_RegisterInterface.readReg16(Regs::FifoStatus);
		const auto available_axes = fifo_status & FIFO_SAMPLES_MASK;
		const auto fifo_bytes = available_axes * FullFifoEntrySize;
		if (fifo_status & FIFO_OVERRUN_LATCHED_MASK) {
			// FIFO overrun is expected to happen during startup and calibration
			m_Logger.error(
				"FIFO OVERRUN! This occurring during normal usage is an issue."
			);
		}

		std::array<uint8_t, FullFifoEntrySize * MaxReadings> read_buffer;
		const auto bytes_to_read = std::min(
									   static_cast<size_t>(read_buffer.size()),
									   static_cast<size_t>(fifo_bytes)
								   )
								 / FullFifoEntrySize * FullFifoEntrySize;
		m_RegisterInterface
			.readBytes(Regs::FifoData, bytes_to_read, read_buffer.data());
		for (auto i = 0u; i < bytes_to_read; i += FullFifoEntrySize) {
			FifoEntryAligned entry;
			uint8_t tag = read_buffer[i] >> 3;
			memcpy(
				entry.raw,
				&read_buffer[i + 0x1],
				sizeof(FifoEntryAligned)
			);  // Skip FIFO header

			switch (tag) {
				case 0x01:  // Gyro NC
					callbacks.processGyroSample(entry.xyz, GyrTs);
					break;
				case 0x02:  // Accel NC
					callbacks.processAccelSample(entry.xyz, AccTs);
					break;
				case 0x03:  // Temperature
					callbacks.processTempSample(entry.xyz[0], TempTs);
					break;
				case 0x13:  // SFLP game rotation vector
					if (callbacks.processSflpGameRotationSample) {
						callbacks.processSflpGameRotationSample(
							decodeSflpGameRotation(entry),
							SflpTs
						);
					}
					break;
			}
		}
		return fifo_bytes > bytes_to_read;
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
