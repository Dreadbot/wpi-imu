// Robot.cpp

#include "Robot.h"

namespace dreadbot {
	class Robot: public IterativeRobot {
	private:
		DriverStation *ds;
		Joystick* driverStick;
		PowerDistributionPanel* pdp;
		MecanumDrive* drivebase;

		MPU6050* IMU;


	public:
		void RobotInit() {
			ds = DriverStation::GetInstance();
			SmartDashboard::init();
			pdp = new PowerDistributionPanel();
			drivebase = new MecanumDrive(1, 2, 3, 4);
			IMU = new MPU6050();
			driveStick = new Joystick(COM_PRIMARY_DRIVER);
			//DreadNav::Initialze();
		}


		void AutonomousInit() {
			Drivebase->Engage();
		}

		void AutonomousPeriodic() { }


		void TeleopInit() {
			Drivebase->Engage();
			DreadNav::Start();
		}

		void TeleopPeriodic() {
			int16_t acc[3] = {0, 0, 0};
			int16_t gyr[3] = {0, 0, 0};
			int16_t mag[3] = {0, 0, 0};

			IMU->getMotion9(
				acc, acc + 1, acc + 2, 
				gyr, gyr + 1, gyr + 2, 
				mag, mag + 1, mag + 2
			);
			
			SmartDashboard::PutNumber("mpu/acc", ax);
			SmartDashboard::PutNumber("mpu/gyr", ax);
			SmartDashboard::PutNumber("mpu/mag", ax);

			//drivebase->SD_OutputDiagnostics();
			float driveX = DRIVE_MULT_STR*CurveInput(driverStick->GetRawAxis(AXS_DRIVE_X));
			float driveY = DRIVE_MULT_FWD*CurveInput(driverStick->GetRawAxis(AXS_DRIVE_Y));
			float driveR = DRIVE_MULT_ROT*CurveInput(driverStick->GetRawAxis(AXS_DRIVE_R));
			
			drivebase->Drive(driveX, driveY, driveR);
		}


		void TestInit() {}

		void TestPeriodic() {}


		void DisabledInit() {
			drivebase->Disengage();
		}

		void DisabledPeriodic() {}


		static float CurveInput(float x) {
			float y = 0.255000975f*std::exp2(2.299113817f*std::fabs(x));
			// coefficients are calculated to imitate the slope of a square function at x=1 with 
			/// ie. solve for a & b where 1=a*e^b & 2=a*e^(b)/b
			if (std::fabs(x) < DRIVER_DEADZONE)
				return 0.0f;
			else
				return x < 0.0f ? -y : y;
		}
	};
}

START_ROBOT_CLASS(dreadbot::Robot);
