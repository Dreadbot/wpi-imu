//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
// 08/15/2014	P Stebbins		Update normalization algorithm
// 02/02/2015	P Stebbins		Classes for quaternion and Tait-Bryan maths
//
//=====================================================================================================


#pragma once


#include <cmath>
#include <stdint.h>

#define sqr(x) x*x
#define AHRS_REPORT_ERRS


/**
 * Quaternion container
 */
struct Quaternion {
public:
	float q0, q1, q2, q3;

	// Default constructor
	explicit Quaternion()
		: q0(1.f), q1(0.f), q2(0.f), q3(0.f) { }

	// Construct from quaternion values
	explicit Quaternion(float _q0, float _q1, float _q2, float _q3)
		: q0(_q0), q1(_q1), q2(_q2), q3(_q3) { }

	bool GimbalLock() {
		return std::abs(2*(q1*q3 + q0*q2)) < 0.01f;
	}
};


/**
 * Tait-Bryan container
 */
struct TaitBryan {
public:
	float yaw, pitch, roll;

	// Initialize to zero if no arguments are supplied
	explicit TaitBryan()
		: yaw(0.f), pitch(0.f), roll(0.f) { }

	// Convert quaternion to angles
	void Set(Quaternion q) {
		yaw = CalculateYaw(q);
		pitch = CalculatePitch(q);
		roll = CalculateRoll(q);
	}

	static float CalculateYaw(Quaternion q) {
		return std::atan2(q.q1*q.q2 + q.q0*q.q3, 0.5 - (sqr(q.q2) + sqr(q.q3)));
	}
	static float CalculatePitch(Quaternion q) {
		return std::asin(2*(q.q0*q.q2 - q.q1*q.q3));
	}
	static float CalculateRoll(Quaternion q) {
		return std::atan2(q.q2*q.q3 + q.q0*q.q1, 0.5 - (sqr(q.q1) + sqr(q.q2)));
	}
};




class MadgwickFilter {
	float sampleFreq; // samples/second. 512 is default
	float beta; // 2 * proportional gain. 0.1 is default

	Quaternion q; // quaternion of sensor frame relative to auxiliary frame (output value)
	TaitBryan tb;

	static float invSqrt(float x) {
		uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
		float tmp = *(float*)&i;
		return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
	}

public:
	//
	explicit MadgwickFilter(float _sampleFreq = 512.f, float _beta = 0.1f)
		: sampleFreq(_sampleFreq), beta(_beta), q() { }

	void AHRSupdate9DOF(float ax, float ay, float az,
						float gx, float gy, float gz,
						float mx, float my, float mz);

	void AHRSupdate6DOF(float ax, float ay, float az,
						float gx, float gy, float gz);

	float GetYaw() {
		tb.Set(q);
		return TaitBryan::CalculateYaw(q);
	}
};
