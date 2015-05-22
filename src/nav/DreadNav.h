#pragma once

#include <cmath>
#include <string>
#include <WPILib.h>
#include "MPU6050.h"


#define ACCEL_LS_SCALE	MPU6050_ACCEL_FS_2
#define GYRO_LS_SCALE 	MPU6050_GYRO_FS_250
#define OUTPUT_IMU_DATA


namespace dreadbot {

	static std::string channelNames_acc[3] = {"acc-X", "acc-Y", "acc-Z"};
	static std::string channelNames_gyr[3] = {"gyr-X", "gyr-Y", "gyr-Z"};
	static std::string channelNames_mag[3] = {"mag-X", "mag-Y", "mag-Z"};

	template <typename T>
	struct imuOut {

		T acc[3] = {(T) 0, (T) 0, (T) 0};
		T gyr[3] = {(T) 0, (T) 0, (T) 0};
		T mag[3] = {(T) 0, (T) 0, (T) 0};

		void reset() {
			for (uint8_t i = 0; i <= 2; ++i) {
				acc[i] = (T) 0;
				gyr[i] = (T) 0;
				mag[i] = (T) 0;
			}
		}

		std::string toString() {
			std::string out = "A=[";
			for (uint8_t i = 0; i <= 2; ++i) {
				out += std::to_string(acc[i]);
				if (i < 2) out += ", ";
			}
			out += "]  G=[";
			for (uint8_t i = 0; i <= 2; ++i) {
				out += std::to_string(acc[i]);
				if (i < 2) out += ", ";
			}
			out += "]  M=[";
			for (uint8_t i = 0; i <= 2; ++i) {
				out += std::to_string(acc[i]);
				if (i < 2) out += ", ";
			}
			out += "]";
			return out;
		}
	};

	class DreadNav {
	public:
		// Initialize the hardware and data structures.
		static void Initialize() {
			if (!isReady) {
				trackingLoop = new Notifier(DreadNav::CallTracker);
				trackingLoop->StartPeriodic(0.01);
				loopTimer = new Timer();
				IMU = new MPU6050();
				IMU->initialize((uint8_t) ACCEL_LS_SCALE, (uint8_t) GYRO_LS_SCALE);
				#ifdef OUTPUT_IMU_DATA
					SmartDashboard::PutString("MPU-9150", currentData->toString());
					SmartDashboard::PutBoolean("nav-active", false);
					SmartDashboard::PutNumber("mpu-device-id", IMU->getDeviceID());
					SmartDashboard::PutBoolean("mpu-connection", IMU->testConnection());
				#endif // OUTPUT_IMU_DATA
				isReady = true;
			}
		}

		// Perform a reset and start the accumulator.
		static void Start() {
			if (!isRunning && isReady) {
				isRunning = true;
				loopTimer->Start();
				currentData->reset();
				#ifdef OUTPUT_IMU_DATA
					SmartDashboard::PutString("MPU-9150", currentData->toString());
					SmartDashboard::PutBoolean("nav-active", true);
				#endif // OUTPUT_IMU_DATA
			}
		}

		// Stops the accumulator
		static void Stop() {
			if (isRunning && isReady) {
				isRunning = false;
				loopTimer->Stop();
				currentData->reset();
				#ifdef OUTPUT_IMU_DATA
					SmartDashboard::PutString("MPU-9150", currentData->toString());
					SmartDashboard::PutBoolean("nav-active", false);
				#endif // OUTPUT_IMU_DATA
			}
		}

		// Returns true if the acculator is running
		static bool IsRunning() {
			return isRunning;
		}

		// Get the current measurement data
		static imuOut<float> GetData() {
			return *currentData;
		}


	private:
		static void RunCalculations() {
			if (isRunning) {
				//double dt = loopTimer->Get();
				loopTimer->Reset();
				loopTimer->Start();

				float fFactor;

				imuOut<int16_t> rawOut;
				imuOut<float> prcOut;

				IMU->getMotion9(
					rawOut.acc, rawOut.acc + 1, rawOut.acc + 2, 
					rawOut.gyr, rawOut.gyr + 1, rawOut.gyr + 2, 
					rawOut.mag, rawOut.mag + 1, rawOut.mag + 2
				);

				// Poll the sensor to verify
				// range. Default to the initial
				// range if something goes wrong.

				// Process raw accelerometer data
				/* switch (IMU->getFullScaleAccelRange()) {
					case MPU6050_ACCEL_FS_2:
						fFactor = g_fMPU9150AccFactors[0];
						break;
					case MPU6050_ACCEL_FS_4:
						fFactor = g_fMPU9150AccFactors[1];
						break;
					case MPU6050_ACCEL_FS_8:
						fFactor = g_fMPU9150AccFactors[2];
						break;
					case MPU6050_ACCEL_FS_16:
						fFactor = g_fMPU9150AccFactors[3];
						break;
					default:
						fFactor = g_fMPU9150AccFactors[ACCEL_LS_SCALE];
						break;
				} */

				fFactor = g_fMPU9150AccFactors[IMU->getFullScaleAccelRange()];
				prcOut.acc[0] = (float) rawOut.acc[0]*fFactor;
				prcOut.acc[1] = (float) rawOut.acc[1]*fFactor;
				prcOut.acc[2] = (float) rawOut.acc[2]*fFactor;

				// Process raw data from gyro
				/* switch (IMU->getFullScaleGyroRange()) {
					case MPU6050_GYRO_FS_250:
						fFactor = g_fMPU9150GyroFactors[3];
						break;
					case MPU6050_GYRO_FS_500:
						fFactor = g_fMPU9150GyroFactors[3];
						break;
					case MPU6050_GYRO_FS_1000:
						fFactor = g_fMPU9150GyroFactors[3];
						break;
					case MPU6050_GYRO_FS_2000:
						fFactor = g_fMPU9150GyroFactors[3];
						break;
					default:
						fFactor = g_fMPU9150GyroFactors[MPU6050_GYRO_FS_250];
						break;
				} */

				fFactor = g_fMPU9150GyroFactors[IMU->getFullScaleGyroRange()];
				prcOut.gyr[0] = (float) rawOut.gyr[0]*fFactor;
				prcOut.gyr[1] = (float) rawOut.gyr[1]*fFactor;
				prcOut.gyr[2] = (float) rawOut.gyr[2]*fFactor;

				// Process raw magnetometer data
				prcOut.mag[0] = (float) rawOut.mag[0]*g_fMPU9150MagFactor;
				prcOut.mag[1] = (float) rawOut.mag[1]*g_fMPU9150MagFactor;
				prcOut.mag[2] = (float) rawOut.mag[2]*g_fMPU9150MagFactor;


				#ifdef OUTPUT_IMU_DATA
					SmartDashboard::PutString("MPU-9150", currentData->toString());
					SmartDashboard::PutBoolean("nav-active", false);
				#endif // OUTPUT_IMU_DATA
			}
		}

		static void CallTracker(void*) {
			DreadNav::RunCalculations();
		}

		// Convert raw ouput from MPU-9150 accelerometer into m/s^2
		static constexpr float g_fMPU9150AccFactors[] = {
			0.0005985482f,     // Range = +/- 2 g (16384 lsb/g)
			0.0011970964f,     // Range = +/- 4 g (8192 lsb/g)
			0.0023941928f,     // Range = +/- 8 g (4096 lsb/g)
			0.0047883855f      // Range = +/- 16 g (2048 lsb/g)
		};

		// Convert raw ouput from MPU-9150 gyroscope into rad/s
		static constexpr float g_fMPU9150GyroFactors[] = {
			1.3323124e-4f,     // Range = +/- 250 dps (131.0)
			2.6646248e-4f,     // Range = +/- 500 dps (65.5)
			5.3211258e-4f,     // Range = +/- 1000 dps (32.8)
			0.0010642252f      // Range = +/- 2000 dps (16.4)
		};

		// Convert raw ouput from MPU-9150 compass to uT
		static constexpr float g_fMPU9150MagFactor = 0.3f;

		static bool isReady;
		static bool isRunning;

		static imuOut<float>* currentData;
		static Timer* loopTimer;
		static Notifier* trackingLoop;
		static MPU6050* IMU;
	};

	bool DreadNav::isReady = false;
	bool DreadNav::isRunning = false;
	imuOut<float>* DreadNav::currentData;
	Timer* DreadNav::loopTimer;
	Notifier* DreadNav::trackingLoop;
	MPU6050* DreadNav::IMU;
}
