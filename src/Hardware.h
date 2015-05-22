#pragma once

#include "drive/MecanumDrive.h"

namespace dreadbot {
	struct Hardware {
		static void init();

		static MecanumDrive* drivebase;
		// Other actuators go here
	};
}
