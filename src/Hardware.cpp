#include "Hardware.h"

namespace dreadbot {
	void Hardware::init() {
		Hardware::drivebase = new MecanumDrive(1, 2, 3, 4);
	}
	MecanumDrive* Hardware::drivebase;
}
