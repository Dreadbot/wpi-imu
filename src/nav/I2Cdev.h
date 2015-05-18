// I2Cdev.h

// Adapted from Jeff Rowberg's excellent I2C library <https://github.com/sparkfun/MPU-9150_Breakout/tree/master/firmware>
// See the license.txt file in the root directory for licensing info.

#pragma once

#include "WPILib.h"
#include <string>

#define MPU9150_I2C_ADR_LOW 0x68  // invensense eval board
#define MPU9150_I2C_ADR_HIGH 0x69

#define MPU9150_I2C_ADR MPU9150_I2C_ADR_LOW // If all else fails, change this


namespace dreadbot {
	class I2Cdev {
		public:
			I2Cdev();
			static void initialize();

			static int8_t readBit(uint8_t _devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
			static int8_t readBits(uint8_t _devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
			static int8_t readByte(uint8_t _devAddr, uint8_t regAddr, uint8_t *data);
			static int8_t readBytes(uint8_t _devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);

			static bool writeBit(uint8_t _devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
			static bool writeBits(uint8_t _devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
			static bool writeByte(uint8_t _devAddr, uint8_t regAddr, uint8_t data);
			static bool writeBytes(uint8_t _devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);

			const static uint8_t devAddr = MPU9150_I2C_ADR;
			static I2C* device;

		private:
			const static uint16_t readTimeout = 1000;
	};
}
