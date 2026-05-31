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
#include <cstdint>

#include "lsm6ds-common.h"
#include "vqf.h"

#ifndef LSM6DSV_SFLP_EXPERIMENT
#define LSM6DSV_SFLP_EXPERIMENT 0
#endif

#ifndef LSM6DSV_SFLP_ODR_HZ
#define LSM6DSV_SFLP_ODR_HZ 120
#endif

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses the 4g acceleration range and the 1000dps gyroscope range.
// Gyroscope ODR = 240Hz, accelerometer ODR = 120Hz.

struct LSM6DSV : LSM6DSOutputHandler {
	static constexpr uint8_t Address = 0x6a;
	static constexpr auto Name = "LSM6DSV";
	static constexpr auto Type = SensorTypeID::LSM6DSV;

	static constexpr float GyrFreq = 240;
	static constexpr float AccFreq = 120;
	static constexpr float MagFreq = 120;
	static constexpr float TempFreq = 60;

	static constexpr float GyrTs = 1.0 / GyrFreq;
	static constexpr float AccTs = 1.0 / AccFreq;
	static constexpr float MagTs = 1.0 / MagFreq;
	static constexpr float TempTs = 1.0 / TempFreq;
	static constexpr bool UsesSflp = LSM6DSV_SFLP_EXPERIMENT != 0;
	static constexpr float SflpFreq = LSM6DSV_SFLP_ODR_HZ;
	static constexpr float SflpTs = 1.0 / SflpFreq;

	static constexpr float GyroSensitivity = 1000 / 35.0f;
	static constexpr float AccelSensitivity = 1000 / 0.122f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 256.0f;

	static constexpr float TemperatureZROChange = 16.667f;

	static constexpr VQFParams SensorVQFParams{};
	static constexpr size_t MaxFifoReadings = 16;

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x0f;
			static constexpr uint8_t value = 0x70;
		};
		struct FuncCfgAccess {
			static constexpr uint8_t reg = 0x01;
			static constexpr uint8_t valueMainBank = 0;
			static constexpr uint8_t valueEmbeddedFuncBank = (1 << 7);
		};
		struct HAODRCFG {
			static constexpr uint8_t reg = 0x62;
			static constexpr uint8_t value = (0b00);  // 1st ODR table
		};
		struct Ctrl1XLODR {
			static constexpr uint8_t reg = 0x10;
			static constexpr uint8_t value = (0b0010110);  // 120Hz, HAODR
		};
		struct Ctrl2GODR {
			static constexpr uint8_t reg = 0x11;
			static constexpr uint8_t value = (0b0010111);  // 240Hz, HAODR
		};
		struct Ctrl3C {
			static constexpr uint8_t reg = 0x12;
			static constexpr uint8_t valueSwReset = 1;
			static constexpr uint8_t value = (1 << 6) | (1 << 2);  // BDU = 1, IF_INC =
																   // 1
		};
		struct Ctrl6GFS {
			static constexpr uint8_t reg = 0x15;
			static constexpr uint8_t value = (0b0011);  // 1000dps
		};
		struct Ctrl8XLFS {
			static constexpr uint8_t reg = 0x17;
			static constexpr uint8_t value = (0b01);  // 4g
		};
		struct FifoCtrl3BDR {
			static constexpr uint8_t reg = 0x09;
			static constexpr uint8_t value
				= UsesSflp
					? 0
					: 0b01110110;  // Gyroscope at 240Hz, Accel at 120Hz
		};
		struct FifoCtrl4Mode {
			static constexpr uint8_t reg = 0x0a;
			static constexpr uint8_t value = (0b110110);  // continuous mode,
															  // temperature at 60Hz
		};
		struct EmbFuncEnA {
			static constexpr uint8_t reg = 0x04;
			static constexpr uint8_t maskSflpGame = (1 << 1);
			static constexpr uint8_t valueSflpGame = (1 << 1);
		};
		struct EmbFuncFifoEnA {
			static constexpr uint8_t reg = 0x44;
			static constexpr uint8_t maskSflpGame = (1 << 1);
			static constexpr uint8_t valueSflpGame = (1 << 1);
		};
		struct SflpOdr {
			static constexpr uint8_t reg = 0x5e;
			static constexpr uint8_t mask = (0b111 << 3);
			static constexpr uint8_t value =
#if LSM6DSV_SFLP_ODR_HZ == 15
				(0 << 3)
#elif LSM6DSV_SFLP_ODR_HZ == 30
				(1 << 3)
#elif LSM6DSV_SFLP_ODR_HZ == 60
				(2 << 3)
#elif LSM6DSV_SFLP_ODR_HZ == 120
				(3 << 3)
#elif LSM6DSV_SFLP_ODR_HZ == 240
				(4 << 3)
#elif LSM6DSV_SFLP_ODR_HZ == 480
				(5 << 3)
#else
#error "LSM6DSV_SFLP_ODR_HZ must be one of 15, 30, 60, 120, 240, 480"
#endif
				;
		};

		static constexpr uint8_t FifoStatus = 0x1b;
		static constexpr uint8_t FifoData = 0x78;
	};

	LSM6DSV(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: LSM6DSOutputHandler(registerInterface, logger) {}

	void setEmbeddedFunctionBank(bool enabled) {
		m_RegisterInterface.writeReg(
			Regs::FuncCfgAccess::reg,
			enabled ? Regs::FuncCfgAccess::valueEmbeddedFuncBank
					: Regs::FuncCfgAccess::valueMainBank
		);
	}

	void updateEmbeddedFunctionReg(uint8_t reg, uint8_t mask, uint8_t value) {
		const uint8_t current = m_RegisterInterface.readReg(reg);
		const uint8_t next = static_cast<uint8_t>((current & ~mask) | (value & mask));
		m_RegisterInterface.writeReg(reg, next);
	}

	void enableSflp() {
		setEmbeddedFunctionBank(true);
		updateEmbeddedFunctionReg(
			Regs::EmbFuncEnA::reg,
			Regs::EmbFuncEnA::maskSflpGame,
			Regs::EmbFuncEnA::valueSflpGame
		);
		updateEmbeddedFunctionReg(
			Regs::SflpOdr::reg,
			Regs::SflpOdr::mask,
			Regs::SflpOdr::value
		);
		updateEmbeddedFunctionReg(
			Regs::EmbFuncFifoEnA::reg,
			Regs::EmbFuncFifoEnA::maskSflpGame,
			Regs::EmbFuncFifoEnA::valueSflpGame
		);
		setEmbeddedFunctionBank(false);
	}

	bool initialize() {
		// Reset and configure the sensor.
		m_RegisterInterface.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::valueSwReset);
		delay(20);
		m_RegisterInterface.writeReg(Regs::HAODRCFG::reg, Regs::HAODRCFG::value);
		m_RegisterInterface.writeReg(Regs::Ctrl1XLODR::reg, Regs::Ctrl1XLODR::value);
		m_RegisterInterface.writeReg(Regs::Ctrl2GODR::reg, Regs::Ctrl2GODR::value);
		m_RegisterInterface.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::value);
		m_RegisterInterface.writeReg(Regs::Ctrl6GFS::reg, Regs::Ctrl6GFS::value);
		m_RegisterInterface.writeReg(Regs::Ctrl8XLFS::reg, Regs::Ctrl8XLFS::value);
		if constexpr (UsesSflp) {
			enableSflp();
		}
		m_RegisterInterface.writeReg(
			Regs::FifoCtrl3BDR::reg,
			Regs::FifoCtrl3BDR::value
		);
		m_RegisterInterface.writeReg(
			Regs::FifoCtrl4Mode::reg,
			Regs::FifoCtrl4Mode::value
		);
		return true;
	}

	bool bulkRead(DriverCallbacks<int16_t>&& callbacks) {
		return LSM6DSOutputHandler::template bulkRead<Regs, MaxFifoReadings>(
			std::move(callbacks),
			GyrTs,
			AccTs,
			TempTs,
			SflpTs
		);
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
