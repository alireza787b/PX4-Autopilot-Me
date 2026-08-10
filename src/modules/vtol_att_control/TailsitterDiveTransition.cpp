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

#include "TailsitterDiveTransition.hpp"

#include <cmath>
#include <float.h>

#include <mathlib/mathlib.h>

void TailsitterDiveTransition::start(const Config &config, float initial_tilt, float initial_rate)
{
	_config = config;
	_config.max_dive_angle = math::max(_config.max_dive_angle, 0.f);
	_config.handoff_tilt = math::constrain(_config.handoff_tilt, 0.f, kHorizon);
	_config.transition_rate = math::max(_config.transition_rate, FLT_EPSILON);
	_config.transition_acceleration = math::max(_config.transition_acceleration, FLT_EPSILON);
	_config.abort_rate = math::max(_config.abort_rate, FLT_EPSILON);
	_config.abort_acceleration = math::max(_config.abort_acceleration, FLT_EPSILON);
	_config.recovery_acceleration = math::max(_config.recovery_acceleration, FLT_EPSILON);
	_config.airspeed_dwell = math::max(_config.airspeed_dwell, 0.f);
	_config.airspeed_hysteresis = math::max(_config.airspeed_hysteresis, 0.f);

	const float maximum_tilt = kHorizon + _config.max_dive_angle;
	_tilt = math::constrain(initial_tilt, 0.f, maximum_tilt);
	_tilt_rate = math::constrain(initial_rate, -_config.abort_rate, _config.transition_rate);
	_phase = _tilt >= kHorizon ? Phase::Dive : Phase::Rotate;
	_abort_reason = AbortReason::None;
	_airspeed_dwell_time = 0.f;
	_tracking_failure_time = 0.f;
	_allocation_failure_time = 0.f;
	_required_recovery_height = _config.recovery_margin;
}

void TailsitterDiveTransition::startAbort(float actual_tilt, float actual_rate, AbortReason reason)
{
	const float maximum_tilt = kHorizon + _config.max_dive_angle;
	_tilt = math::constrain(actual_tilt, 0.f, maximum_tilt);
	_tilt_rate = math::constrain(actual_rate, -_config.abort_rate, _config.abort_rate);

	// A bounded position trajectory cannot preserve velocity directed out of
	// its range. Start from rest at that boundary instead of introducing the
	// same discontinuity on the first update.
	if ((_tilt <= 0.f && _tilt_rate < 0.f) || (_tilt >= maximum_tilt && _tilt_rate > 0.f)) {
		_tilt_rate = 0.f;
	}

	_phase = Phase::Abort;
	_abort_reason = reason;
}

float TailsitterDiveTransition::requiredRecoveryHeight(float tilt, float vertical_speed_down, float abort_rate,
		float fixed_margin, float recovery_acceleration)
{
	if (!PX4_ISFINITE(tilt) || !PX4_ISFINITE(vertical_speed_down) || !PX4_ISFINITE(abort_rate)
	    || !PX4_ISFINITE(fixed_margin) || !PX4_ISFINITE(recovery_acceleration)
	    || abort_rate <= FLT_EPSILON || recovery_acceleration <= FLT_EPSILON) {
		return INFINITY;
	}

	const float down_speed = math::max(vertical_speed_down, 0.f);
	const float below_horizon_angle = math::max(tilt - kHorizon, 0.f);
	const float time_to_upward_thrust = below_horizon_angle / abort_rate;

	return math::max(fixed_margin, 0.f) + down_speed * time_to_upward_thrust
	       + down_speed * down_speed / (2.f * recovery_acceleration);
}

TailsitterDiveTransition::AbortReason TailsitterDiveTransition::evaluateSafety(const Input &input,
		float required_recovery_height)
{
	if (!input.airspeed_valid || !PX4_ISFINITE(input.airspeed)) {
		return AbortReason::AirspeedInvalid;
	}

	if (!PX4_ISFINITE(input.available_recovery_height) || !PX4_ISFINITE(input.vertical_speed_down)
	    || input.available_recovery_height <= required_recovery_height) {
		return AbortReason::RecoveryAltitude;
	}

	if (!PX4_ISFINITE(input.attitude_error) || !PX4_ISFINITE(input.angular_rate)) {
		return AbortReason::AttitudeTracking;
	}

	if (input.attitude_error > _config.attitude_error_limit) {
		_tracking_failure_time += input.dt;

	} else {
		_tracking_failure_time = 0.f;
	}

	if (_tracking_failure_time >= kTrackingFailureDwell) {
		return AbortReason::AttitudeTracking;
	}

	if (input.allocation_status_valid && PX4_ISFINITE(input.unallocated_pitch_torque)
	    && fabsf(input.unallocated_pitch_torque) > _config.allocation_error_limit) {
		_allocation_failure_time += input.dt;

	} else {
		_allocation_failure_time = 0.f;
	}

	if (_allocation_failure_time >= kAllocationFailureDwell) {
		return AbortReason::ControlAllocation;
	}

	return AbortReason::None;
}

void TailsitterDiveTransition::updateTrajectory(float target, float rate_limit, float acceleration_limit, float dt)
{
	dt = math::constrain(dt, 0.0001f, 0.1f);
	rate_limit = math::max(rate_limit, FLT_EPSILON);
	acceleration_limit = math::max(acceleration_limit, FLT_EPSILON);

	const float distance = target - _tilt;
	// Account for one integration interval when calculating the braking
	// velocity. This prevents a finite-rate snap when the target is reached.
	const float current_rate = fabsf(_tilt_rate);
	const float braking_reserve = current_rate * current_rate / (2.f * acceleration_limit)
				      + current_rate * dt;
	const float stopping_rate = sqrtf(2.f * acceleration_limit * math::max(fabsf(distance) - braking_reserve, 0.f));
	const float desired_rate = math::signNoZero(distance) * math::min(rate_limit, stopping_rate);
	const float previous_rate = _tilt_rate;
	_tilt_rate = math::constrain(desired_rate, _tilt_rate - acceleration_limit * dt,
				     _tilt_rate + acceleration_limit * dt);

	const float previous_distance = distance;
	_tilt += 0.5f * (previous_rate + _tilt_rate) * dt;
	const float new_distance = target - _tilt;

	if (previous_distance * new_distance <= 0.f) {
		_tilt = target;
	}

	const float maximum_tilt = kHorizon + _config.max_dive_angle;
	_tilt = math::constrain(_tilt, 0.f, maximum_tilt);

	// Keep the internal rate while the bounded position is held at an end
	// point. It is then decelerated on the following update, avoiding a rate
	// step caused solely by clamping the attitude setpoint.
}

TailsitterDiveTransition::Output TailsitterDiveTransition::output() const
{
	Output result{};
	result.phase = _phase;
	result.abort_reason = _abort_reason;
	result.tilt = _tilt;
	result.tilt_rate = _tilt_rate;
	result.required_recovery_height = _required_recovery_height;
	result.transition_complete = _phase == Phase::Complete;
	result.recovery_complete = _phase == Phase::Recovered;
	return result;
}

TailsitterDiveTransition::Output TailsitterDiveTransition::update(const Input &input)
{
	if (_phase == Phase::Inactive || _phase == Phase::Complete || _phase == Phase::Recovered) {
		return output();
	}

	if (_phase == Phase::Abort) {
		updateTrajectory(0.f, _config.abort_rate, _config.abort_acceleration, input.dt);

		if (_tilt <= kPositionTolerance && fabsf(_tilt_rate) <= kRateTolerance
		    && PX4_ISFINITE(input.attitude_error) && input.attitude_error <= math::radians(10.f)
		    && PX4_ISFINITE(input.angular_rate) && input.angular_rate <= _config.handoff_rate_limit) {
			_tilt = 0.f;
			_tilt_rate = 0.f;
			_phase = Phase::Recovered;
		}

		return output();
	}

	_required_recovery_height = requiredRecoveryHeight(math::max(_tilt, input.actual_tilt),
				    input.vertical_speed_down, _config.abort_rate,
				    _config.recovery_margin, _config.recovery_acceleration);

	const AbortReason safety_reason = evaluateSafety(input, _required_recovery_height);

	if (safety_reason != AbortReason::None) {
		Output result = output();
		result.abort_reason = safety_reason;
		result.request_abort = true;
		return result;
	}

	if (input.airspeed >= _config.transition_airspeed) {
		_airspeed_dwell_time += input.dt;

	} else if (input.airspeed < _config.transition_airspeed - _config.airspeed_hysteresis) {
		_airspeed_dwell_time = 0.f;

		if (_phase == Phase::Capture) {
			Output result = output();
			result.abort_reason = AbortReason::AirspeedInvalid;
			result.request_abort = true;
			return result;
		}
	}

	if (_airspeed_dwell_time >= _config.airspeed_dwell) {
		_phase = Phase::Capture;
	}

	const float target = _phase == Phase::Capture
			     ? _config.handoff_tilt
			     : kHorizon + _config.max_dive_angle;
	updateTrajectory(target, _config.transition_rate, _config.transition_acceleration, input.dt);

	if (_phase == Phase::Rotate && _tilt >= kHorizon) {
		_phase = Phase::Dive;
	}

	if (_phase == Phase::Capture && input.elapsed >= _config.minimum_transition_time
	    && fabsf(_tilt - _config.handoff_tilt) <= kPositionTolerance
	    && fabsf(_tilt_rate) <= kRateTolerance
	    && input.attitude_error <= math::radians(10.f)
	    && input.angular_rate <= _config.handoff_rate_limit
	    && input.airspeed >= _config.transition_airspeed - _config.airspeed_hysteresis
	    && input.handoff_ready) {
		_tilt = _config.handoff_tilt;
		_tilt_rate = 0.f;
		_phase = Phase::Complete;
	}

	return output();
}
