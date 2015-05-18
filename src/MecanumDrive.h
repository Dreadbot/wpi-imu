// MecanumDrive.h


#pragma once

#include <WPILib.h>
#include <algorithm>
#include <cmath>
#include <functional>

#define MOTOR_COUNT 4		// Number of motors
#define CONTROL_PERIOD 6	// Periodic update delay in ms

namespace dreadbot {
	class MecanumDrive {
	public:
		enum class drivemode {
			absolute, // Position control
			relative  // Velocity control
		};

		enum motorType : uint8_t {
			m_leftFront = 0,
			m_rightFront = 1,
			m_leftRear = 2,
			m_rightRear = 3
		};

		MecanumDrive(int motorId_lf, int motorId_rf, int motorId_lr, int motorId_rr);
		~MecanumDrive();

		void GoSlow();
		void GoFast();
		void GoSpeed(double speed);
		void Drive_p(double x, double y, double rotation); //Unimplemented position-based driving.
		void Drive_v(double x, double y, double rotation); //Velocity based driving.
		void SetDriveMode(drivemode newMode);
		
		void Engage();		// Enables driving
		void Disengage();	// Disables driving

		void SD_RetrievePID(); 			// Retrieve control loop constants from SD
		void SD_OutputDiagnostics(); 	// Output motor diagnostic info to SD

	protected:
		bool m_enabled = false;
		const uint8_t syncGroup = 0x00;
		const std::string motorNames[MOTOR_COUNT] = {"LF Drive [1]", "RF Drive [2]", "LB Drive [3]", "RB Drive [4]"};
		const double motorReversals[MOTOR_COUNT] = {-1.0, 1.0, -1.0, 1.0};
		drivemode mode = drivemode::relative;
		CANTalon* motors[4];

	private:
		DISALLOW_COPY_AND_ASSIGN(MecanumDrive);
	};
}
