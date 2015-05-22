// Robot.cpp

#include "Robot.h"

namespace dreadbot {
	class Robot: public IterativeRobot {
	private:
		DriverStation *driverStation;
		Joystick* driveController;
		PowerDistributionPanel* pdp;
		MPU6050* IMU;

	public:
		void RobotInit() {
			driverStation = DriverStation::GetInstance();
			SmartDashboard::init();
			Hardware::init();
			pdp = new PowerDistributionPanel();

			IMU = new MPU6050();
			driveController = new Joystick(COM_PRIMARY_DRIVER);
			//DreadNav::Initialze();
		}


		void AutonomousInit() {
			Hardware::drivebase->Engage();
		}

		void AutonomousPeriodic() { }


		void TeleopInit() {
			Hardware::drivebase->Engage();
			//DreadNav::Start();
		}

		void TeleopPeriodic() {
			imuOut<int16_t> rawOut;

			IMU->getMotion9(
				rawOut.acc, rawOut.acc + 1, rawOut.acc + 2,
				rawOut.gyr, rawOut.gyr + 1, rawOut.gyr + 2,
				rawOut.mag, rawOut.mag + 1, rawOut.mag + 2
			);
			
			SmartDashboard::PutString("MPU out", rawOut.toString());

			Hardware::drivebase->SD_OutputDiagnostics();
			float driveX = DRIVE_MULT_STR*CurveInput(driveController->GetRawAxis(AXIS_DRIVE_X));
			float driveY = DRIVE_MULT_FWD*CurveInput(driveController->GetRawAxis(AXIS_DRIVE_Y));
			float driveR = DRIVE_MULT_ROT*CurveInput(driveController->GetRawAxis(AXIS_DRIVE_R));
			
			Hardware::drivebase->Drive_v(driveX, driveY, driveR);
		}


		void TestInit() {}

		void TestPeriodic() {}


		void DisabledInit() {
			Hardware::drivebase->Disengage();
		}

		void DisabledPeriodic() {}


		static float CurveInput(float input) {
			// This form was selected for its ease of use, ideal endpoint slopes, and computational cheapness.
			float output = 0.255000975f * (std::exp2(2.299113817f * std::fabs(input)) - 1.f);
			// to adjust the curvature distribution, recompute A and B in A*(2^(B*X)-1) so that it fits the full output range
			if (std::fabs(input) < DRIVER_DEADZONE)
				return 0.0f;
			else
				return std::copysign(output, input);
		}
	};
}

START_ROBOT_CLASS(dreadbot::Robot);
