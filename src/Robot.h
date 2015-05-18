// Robot.h

#pragma once

#include <WPILib.h>
#include "MecanumDrive.h"
#include "MPU6050.h"
#include "DreadNav.h"

// Drive constants
#define DRIVE_MULT_FWD 0.7f
#define DRIVE_MULT_STR 0.85f
#define DRIVE_MULT_ROT 0.9f


// COM ports
#define COM_PRIMARY_DRIVER 0


#define LOGITECH_F310_X // Use Logitech F310 controllers

// Drive control mappings for Logitech F310
#ifdef LOGITECH_F310_X
	#define AXIS_DRIVE_X 0
	#define AXIS_DRIVE_Y 1
	#define AXIS_DRIVE_R 4
	#define BTN_STOP_LIFT 1
	#define DRIVER_DEADZONE 0.015
#endif
