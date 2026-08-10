/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

class TailsitterDiveTransition
{
public:
	enum class Phase : uint8_t {
		Inactive = 0,
		Rotate,
		Dive,
		Capture,
		Abort,
		Complete,
		Recovered,
	};

	enum class AbortReason : uint8_t {
		None = 0,
		AirspeedInvalid,
		RecoveryAltitude,
		AttitudeTracking,
		ControlAllocation,
		SetpointStale,
		External,
	};

	struct Config {
		float max_dive_angle{};              // [rad] maximum tilt beyond the 90 degree horizon
		float transition_rate{};             // [rad/s]
		float transition_acceleration{};     // [rad/s^2]
		float abort_rate{};                  // [rad/s]
		float abort_acceleration{};          // [rad/s^2]
		float handoff_tilt{};                // [rad] tilt from hover for FW handoff
		float transition_airspeed{};         // [m/s] calibrated airspeed
		float minimum_transition_time{};     // [s]
		float airspeed_dwell{0.3f};          // [s]
		float airspeed_hysteresis{1.f};      // [m/s]
		float recovery_margin{};             // [m]
		float recovery_acceleration{};       // [m/s^2]
		float attitude_error_limit{};        // [rad]
		float allocation_error_limit{};      // [normalized torque]
		float handoff_rate_limit{};          // [rad/s]
	};

	struct Input {
		float dt{};                          // [s]
		float elapsed{};                     // [s]
		float airspeed{};                    // [m/s]
		float actual_tilt{};                 // [rad] signed tilt from hover
		float attitude_error{};              // [rad] quaternion tracking error
		float angular_rate{};                // [rad/s] body-rate norm
		float available_recovery_height{};   // [m] height above configured FW floor
		float vertical_speed_down{};          // [m/s], positive down
		float unallocated_pitch_torque{};     // [normalized torque]
		bool airspeed_valid{false};
		bool allocation_status_valid{false};
		bool handoff_ready{false};
	};

	struct Output {
		Phase phase{Phase::Inactive};
		AbortReason abort_reason{AbortReason::None};
		float tilt{};                        // [rad] commanded tilt from hover
		float tilt_rate{};                   // [rad/s]
		float required_recovery_height{};    // [m]
		bool request_abort{false};
		bool transition_complete{false};
		bool recovery_complete{false};
	};

	void start(const Config &config, float initial_tilt = 0.f, float initial_rate = 0.f);
	void startAbort(float actual_tilt, float actual_rate, AbortReason reason);
	Output update(const Input &input);

	Phase phase() const { return _phase; }
	float tilt() const { return _tilt; }
	float tiltRate() const { return _tilt_rate; }

	static float requiredRecoveryHeight(float tilt, float vertical_speed_down, float abort_rate,
					    float fixed_margin, float recovery_acceleration);

private:
	static constexpr float kHorizon = 1.57079632679489661923f;
	static constexpr float kTrackingFailureDwell = 0.5f;
	static constexpr float kAllocationFailureDwell = 0.5f;
	static constexpr float kPositionTolerance = 0.00872664626f; // 0.5 deg
	static constexpr float kRateTolerance = 0.00174532925f; // 0.1 deg/s

	void updateTrajectory(float target, float rate_limit, float acceleration_limit, float dt);
	Output output() const;
	AbortReason evaluateSafety(const Input &input, float required_recovery_height);

	Config _config{};
	Phase _phase{Phase::Inactive};
	AbortReason _abort_reason{AbortReason::None};
	float _tilt{0.f};
	float _tilt_rate{0.f};
	float _airspeed_dwell_time{0.f};
	float _tracking_failure_time{0.f};
	float _allocation_failure_time{0.f};
	float _required_recovery_height{0.f};
};
