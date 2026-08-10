/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
* @file tailsitter.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#include "tailsitter.h"
#include "vtol_att_control_main.h"

#include <cmath>

using namespace matrix;

Tailsitter::Tailsitter(VtolAttitudeControl *attc) :
	VtolType(attc)
{
}

bool Tailsitter::diveTransitionEnabled() const
{
	return _param_vt_ts_dive_en.get();
}

void Tailsitter::configureDiveTransition()
{
	_dive_transition_config = {};
	_dive_transition_config.max_dive_angle = math::radians(_param_vt_ts_dive_ang.get());
	_dive_transition_config.transition_rate = math::radians(_param_vt_ts_dive_rat.get());
	// The trajectory is acceleration limited. This reaches the configured rate in
	// approximately 0.5 s and, importantly, keeps the horizon crossing continuous.
	_dive_transition_config.transition_acceleration = 2.f * _dive_transition_config.transition_rate;
	_dive_transition_config.abort_rate = math::radians(_param_vt_ts_abrt_rat.get());
	_dive_transition_config.abort_acceleration = 2.f * _dive_transition_config.abort_rate;
	_dive_transition_config.handoff_tilt = math::constrain(M_PI_2_F -
					       math::radians(_param_fw_psp_off.get()), 0.f, M_PI_2_F);
	_dive_transition_config.transition_airspeed = getTransitionAirspeed();
	_dive_transition_config.minimum_transition_time = getMinimumFrontTransitionTime();
	_dive_transition_config.recovery_margin = _param_vt_ts_rec_alt.get();
	_dive_transition_config.recovery_acceleration = _param_vt_ts_rec_acc.get();
	_dive_transition_config.attitude_error_limit = math::radians(35.f);
	_dive_transition_config.allocation_error_limit = 0.05f;
	_dive_transition_config.handoff_rate_limit = math::radians(30.f);
}

float Tailsitter::actualTransitionTilt() const
{
	if (_trans_rot_axis.norm() < FLT_EPSILON) {
		return 0.f;
	}

	Quatf q_actual(_v_att->q);

	if (!q_actual.isAllFinite() || q_actual.norm() < FLT_EPSILON) {
		return _dive_transition.tilt();
	}

	q_actual.normalize();
	Quatf q_relative = q_actual * _q_trans_start.inversed();
	q_relative.normalize();

	// q and -q represent the same attitude. Use the shortest relative rotation
	// before projecting it onto the captured transition axis.
	if (q_relative(0) < 0.f) {
		q_relative *= -1.f;
	}

	const float tilt = AxisAnglef(q_relative).dot(_trans_rot_axis.unit());
	return math::constrain(tilt, 0.f, M_PI_2_F + _dive_transition_config.max_dive_angle);
}

float Tailsitter::actualTransitionTiltRate() const
{
	if (_trans_rot_axis.norm() < FLT_EPSILON) {
		return 0.f;
	}

	const auto *angular_velocity = _attc->get_angular_velocity();

	if (angular_velocity->timestamp == 0 || hrt_elapsed_time(&angular_velocity->timestamp) > 200_ms) {
		return 0.f;
	}

	const Vector3f body_rate(angular_velocity->xyz);

	if (!body_rate.isAllFinite()) {
		return 0.f;
	}

	const Quatf attitude(_v_att->q);

	if (!attitude.isAllFinite() || attitude.norm() < FLT_EPSILON) {
		return 0.f;
	}

	const Vector3f earth_rate = attitude.rotateVector(body_rate);
	return earth_rate.dot(_trans_rot_axis.unit());
}

float Tailsitter::transitionAttitudeError() const
{
	Quatf q_actual(_v_att->q);
	Quatf q_setpoint(_q_trans_sp);

	if (!q_actual.isAllFinite() || !q_setpoint.isAllFinite()
	    || q_actual.norm() < FLT_EPSILON || q_setpoint.norm() < FLT_EPSILON) {
		return NAN;
	}

	q_actual.normalize();
	q_setpoint.normalize();

	const float dot = math::constrain(fabsf(q_actual.dot(q_setpoint)), 0.f, 1.f);
	return 2.f * acosf(dot);
}

float Tailsitter::availableRecoveryHeight() const
{
	float distance_to_ground = NAN;

	if (_local_pos->dist_bottom_valid && PX4_ISFINITE(_local_pos->dist_bottom)) {
		distance_to_ground = _local_pos->dist_bottom;

	} else if (_local_pos->z_valid && PX4_ISFINITE(_attc->get_home_position_z())) {
		distance_to_ground = -(_local_pos->z - _attc->get_home_position_z());

	} else if (_local_pos->z_valid && PX4_ISFINITE(_local_pos->z)) {
		distance_to_ground = -_local_pos->z;
	}

	if (!PX4_ISFINITE(distance_to_ground)) {
		return NAN;
	}

	return distance_to_ground - math::max(_param_vt_fw_min_alt.get(), 0.f);
}

void Tailsitter::initializeControlledAbort(TailsitterDiveTransition::AbortReason reason)
{
	if (!_dive_transition_initialized || _vtol_mode == vtol_mode::TRANSITION_FRONT_ABORT) {
		return;
	}

	_dive_transition.startAbort(actualTransitionTilt(), actualTransitionTiltRate(), reason);
	_vtol_mode = vtol_mode::TRANSITION_FRONT_ABORT;
	_transition_start_timestamp = hrt_absolute_time();
	_time_since_trans_start = 0.f;
}

void Tailsitter::triggerDiveAbort(TailsitterDiveTransition::AbortReason reason, bool quadchute)
{
	if (quadchute) {
		QuadchuteReason quadchute_reason = QuadchuteReason::TransitionAirspeedInvalid;

		switch (reason) {
		case TailsitterDiveTransition::AbortReason::AirspeedInvalid:
			quadchute_reason = QuadchuteReason::TransitionAirspeedInvalid;
			break;

		case TailsitterDiveTransition::AbortReason::RecoveryAltitude:
			quadchute_reason = QuadchuteReason::TransitionRecoveryAltitude;
			break;

		case TailsitterDiveTransition::AbortReason::AttitudeTracking:
			quadchute_reason = QuadchuteReason::TransitionAttitudeTracking;
			break;

		case TailsitterDiveTransition::AbortReason::ControlAllocation:
			quadchute_reason = QuadchuteReason::TransitionControlAllocation;
			break;

		case TailsitterDiveTransition::AbortReason::SetpointStale:
			quadchute_reason = QuadchuteReason::TransitionSetpointStale;
			break;

		default:
			quadchute_reason = QuadchuteReason::ExternalCommand;
			break;
		}

		_attc->quadchute(quadchute_reason);
	}

	initializeControlledAbort(reason);
}

void Tailsitter::updateDiveTransitionThrust(float commanded_tilt)
{
	const float hover_thrust = math::constrain(_param_mpc_thr_hover.get(), 0.1f, 0.8f);
	float target_thrust = _param_vt_ts_dive_thr.get();

	if (_vtol_mode == vtol_mode::TRANSITION_FRONT_ABORT) {
		const float low_thrust = 0.5f * hover_thrust;

		if (commanded_tilt >= M_PI_2_F) {
			target_thrust = low_thrust;

		} else {
			const float recovery_progress = math::constrain((M_PI_2_F - commanded_tilt) / math::radians(30.f), 0.f, 1.f);
			const bool mc_setpoint_fresh = _mc_virtual_att_sp->timestamp != 0
						       && hrt_elapsed_time(&_mc_virtual_att_sp->timestamp) <= 1_s;
			const float mc_thrust = mc_setpoint_fresh && PX4_ISFINITE(_vehicle_thrust_setpoint_virtual_mc->xyz[2])
						? math::constrain(-_vehicle_thrust_setpoint_virtual_mc->xyz[2], 0.f, 1.f)
						: hover_thrust;
			target_thrust = low_thrust + recovery_progress * (math::max(mc_thrust, hover_thrust) - low_thrust);
		}
	}

	target_thrust = math::constrain(target_thrust, 0.1f, 0.9f);
	const float slew_rate = 2.f; // normalized collective per second
	_dive_transition_thrust = math::constrain(target_thrust,
				  _dive_transition_thrust - slew_rate * _transition_dt,
				  _dive_transition_thrust + slew_rate * _transition_dt);
	_v_att_sp->thrust_body[2] = -_dive_transition_thrust;
}

void Tailsitter::updateDiveTransition()
{
	TailsitterDiveTransition::Input input{};
	input.dt = _transition_dt;
	input.elapsed = _time_since_trans_start;
	input.actual_tilt = actualTransitionTilt();
	input.attitude_error = transitionAttitudeError();
	const auto *angular_velocity = _attc->get_angular_velocity();
	input.angular_rate = (angular_velocity->timestamp != 0
			      && hrt_elapsed_time(&angular_velocity->timestamp) <= 200_ms
			      && PX4_ISFINITE(angular_velocity->xyz[0])
			      && PX4_ISFINITE(angular_velocity->xyz[1])
			      && PX4_ISFINITE(angular_velocity->xyz[2]))
			     ? Vector3f(angular_velocity->xyz).norm() : NAN;
	input.available_recovery_height = availableRecoveryHeight();
	input.vertical_speed_down = (_local_pos->v_z_valid && PX4_ISFINITE(_local_pos->vz)) ? math::max(_local_pos->vz, 0.f) : NAN;

	float physical_airspeed = NAN;
	input.airspeed_valid = _attc->get_fresh_physical_airspeed(physical_airspeed);
	input.airspeed = physical_airspeed;
	input.handoff_ready = _fw_virtual_att_sp->timestamp != 0
			      && hrt_elapsed_time(&_fw_virtual_att_sp->timestamp) <= 1_s;

	const auto *allocator_status = _attc->get_control_allocator_status();
	input.allocation_status_valid = allocator_status->timestamp != 0
					&& hrt_elapsed_time(&allocator_status->timestamp) < 500_ms;
	input.unallocated_pitch_torque = allocator_status->unallocated_torque[1];

	auto output = _dive_transition.update(input);

	if (output.request_abort) {
		triggerDiveAbort(output.abort_reason, true);
		output = _dive_transition.update(input);
	}

	_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis, output.tilt)) * _q_trans_start;
	_q_trans_sp.normalize();
	updateDiveTransitionThrust(output.tilt);

	_v_att_sp->timestamp = hrt_absolute_time();
	_q_trans_sp.copyTo(_v_att_sp->q_d);
}

void
Tailsitter::parameters_update()
{
	VtolType::updateParams();

}

Eulerf Tailsitter::getFixedWingAttitudeEuler() const
{
	// Tailsitter attitude is estimated in MC frame; rotate it to FW frame before checking FW limits.
	return Eulerf(Quatf(_v_att->q) * _q_fw_to_mc);
}

bool Tailsitter::isPitchExceeded()
{
	if (_common_vtol_mode != mode::FIXED_WING) {
		return false;
	}

	// fixed-wing maximum pitch angle
	if (_param_vt_fw_qc_p.get() > 0) {
		const Eulerf euler = getFixedWingAttitudeEuler();

		if (fabsf(euler.theta()) > fabsf(math::radians(static_cast<float>(_param_vt_fw_qc_p.get())))) {
			return true;
		}
	}

	return false;
}

bool Tailsitter::isRollExceeded()
{
	if (_common_vtol_mode != mode::FIXED_WING) {
		return false;
	}

	// fixed-wing maximum roll angle
	if (_param_vt_fw_qc_r.get() > 0) {
		const Eulerf euler = getFixedWingAttitudeEuler();

		if (fabsf(euler.phi()) > fabsf(math::radians(static_cast<float>(_param_vt_fw_qc_r.get())))) {
			return true;
		}
	}

	return false;
}

void Tailsitter::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/


	if (_vtol_vehicle_status->fixed_wing_system_failure) {
		// Keep the opt-in dive transition on the MC controller while it rotates
		// back to hover. The stock path remains an immediate MC switch.
		if (_dive_transition_active && _dive_transition_initialized
		    && (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1 || _vtol_mode == vtol_mode::TRANSITION_FRONT_ABORT)) {
			if (_vtol_mode == vtol_mode::TRANSITION_FRONT_ABORT
			    && _dive_transition.phase() == TailsitterDiveTransition::Phase::Recovered) {
				_vtol_mode = vtol_mode::MC_MODE;
				_dive_transition_active = false;
				_dive_transition_initialized = false;

			} else {
				initializeControlledAbort(TailsitterDiveTransition::AbortReason::External);
			}

		} else {
			if (_vtol_mode != vtol_mode::MC_MODE) {
				_transition_start_timestamp = hrt_absolute_time();
			}

			_vtol_mode = vtol_mode::MC_MODE;
		}

	} else if (!_attc->is_fixed_wing_requested()) {

		switch (_vtol_mode) { // user switchig to MC mode
		case vtol_mode::MC_MODE:
			break;

		case vtol_mode::FW_MODE:
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_BACK;
			break;

		case vtol_mode::TRANSITION_FRONT_P1:
			if (_dive_transition_active && _dive_transition_initialized) {
				// A commanded cancellation is handled by the same bounded
				// quaternion recovery as a transition failure.
				initializeControlledAbort(TailsitterDiveTransition::AbortReason::External);

			} else {
				_vtol_mode = vtol_mode::MC_MODE;
			}

			break;

		case vtol_mode::TRANSITION_FRONT_ABORT:
			if (_dive_transition.phase() == TailsitterDiveTransition::Phase::Recovered) {
				_vtol_mode = vtol_mode::MC_MODE;
				_dive_transition_active = false;
				_dive_transition_initialized = false;
			}

			break;

		case vtol_mode::TRANSITION_BACK:
			const float pitch = Eulerf(Quatf(_v_att->q)).theta();

			// check if we have reached pitch angle to switch to MC mode
			if (pitch >= PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC || _time_since_trans_start > _param_vt_b_trans_dur.get()) {
				_vtol_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {  // user switchig to FW mode

		switch (_vtol_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			_vtol_mode = vtol_mode::TRANSITION_FRONT_P1;
			_dive_transition_active = diveTransitionEnabled();
			_dive_transition_initialized = false;
			_dive_handoff_active = false;
			resetTransitionStates();
			break;

		case vtol_mode::FW_MODE:
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {
				const bool transition_complete = _dive_transition_active
								 ? (_dive_transition.phase() == TailsitterDiveTransition::Phase::Complete)
								 : isFrontTransitionCompleted();

				if (transition_complete) {
					_vtol_mode = vtol_mode::FW_MODE;
					_trans_finished_ts = hrt_absolute_time();

					if (_dive_transition_active) {
						_dive_handoff_active = true;
						_dive_handoff_start_ts = _trans_finished_ts;
						// Capture the outputs that were actually driving the motors in
						// transition. The virtual MC thrust may already have changed
						// while the dive trajectory was active.
						_dive_handoff_thrust = -_dive_transition_thrust;
						_dive_handoff_torque = Vector3f(_vehicle_torque_setpoint_virtual_mc->xyz);
					}
				}

				break;
			}

		case vtol_mode::TRANSITION_FRONT_ABORT:
			// Remain in the controlled recovery even if a stale FW request is
			// still present. Switching to MC is decided by the recovery guard.
			break;

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_mode = vtol_mode::FW_MODE;
			_trans_finished_ts = hrt_absolute_time();
			break;
		}
	}

	// map tailsitter specific control phases to simple control modes
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
		_common_vtol_mode = mode::ROTARY_WING;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::FW_MODE:
		_common_vtol_mode = mode::FIXED_WING;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
		_common_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_FRONT_ABORT:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;

	case vtol_mode::TRANSITION_BACK:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Tailsitter::update_transition_state()
{
	VtolType::update_transition_state();

	const hrt_abstime now = hrt_absolute_time();

	// we need the incoming (virtual) mc attitude setpoints to be recent, otherwise return (means the previous setpoint stays active)
	if (_mc_virtual_att_sp->timestamp < (now - 1_s)) {
		if (_dive_transition_active && _dive_transition_initialized
		    && _vtol_mode != vtol_mode::TRANSITION_FRONT_ABORT) {
			triggerDiveAbort(TailsitterDiveTransition::AbortReason::SetpointStale, true);

		} else if (!_dive_transition_active || !_dive_transition_initialized) {
			return;
		}
	}

	if (!_flag_was_in_trans_mode) {
		_flag_was_in_trans_mode = true;

		if (_vtol_mode == vtol_mode::TRANSITION_BACK) {
			// calculate rotation axis for transition.
			_q_trans_start = Quatf(_v_att->q);
			Vector3f z = -_q_trans_start.dcm_z();
			_trans_rot_axis = z.cross(Vector3f(0.f, 0.f, -1.f));

			// as heading setpoint we choose the heading given by the direction the vehicle points
			const float yaw_sp = atan2f(z(1), z(0));

			// the intial attitude setpoint for a backtransition is a combination of the current fw pitch setpoint,
			// the yaw setpoint and zero roll since we want wings level transition.
			// If for some reason the fw attitude setpoint is not recent then don't use it and assume 0 pitch
			if (_fw_virtual_att_sp->timestamp > (now - 1_s)) {
				const float pitch_body = Eulerf(Quatf(_fw_virtual_att_sp->q_d)).theta();
				_q_trans_start = Eulerf(0.f, pitch_body, yaw_sp);

			} else {
				_q_trans_start = Eulerf(0.f, 0.f, yaw_sp);
			}

			// attitude during transitions are controlled by mc attitude control so rotate the desired attitude to the
			// multirotor frame
			_q_trans_start = _q_trans_start * Quatf(Eulerf(0, -M_PI_2_F, 0));

		} else if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {
			// initial attitude setpoint for the transition should be with wings level
			const Eulerf setpoint_euler(Quatf(_mc_virtual_att_sp->q_d));
			_q_trans_start = Eulerf(0.f, setpoint_euler.theta(), setpoint_euler.psi());
			Vector3f x = Dcmf(Quatf(_v_att->q)) * Vector3f(1.f, 0.f, 0.f);
			_trans_rot_axis = -x.cross(Vector3f(0.f, 0.f, -1.f));

			if (_trans_rot_axis.norm() > 0.1f) {
				_trans_rot_axis.normalize();

			} else {
				// Degenerate only when the estimated body x-axis is nearly
				// vertical. Retain heading by using the setpoint body -y axis.
				_trans_rot_axis = Quatf(_q_trans_start).rotateVector(Vector3f(0.f, -1.f, 0.f));
				_trans_rot_axis.normalize();
			}

			if (_dive_transition_active) {
				configureDiveTransition();
				_dive_transition.start(_dive_transition_config);
				_dive_transition_initialized = true;
				_dive_transition_thrust = math::constrain(-_vehicle_thrust_setpoint_virtual_mc->xyz[2], 0.1f, 0.9f);
			}
		}

		_q_trans_sp = _q_trans_start;
	}

	// ensure input quaternions are exactly normalized because acosf(1.00001) == NaN
	_q_trans_sp.normalize();

	// tilt angle (zero if vehicle nose points up (hover))
	const float cos_tilt = math::constrain(_q_trans_sp(0) * _q_trans_sp(0) - _q_trans_sp(1) * _q_trans_sp(1) -
					       _q_trans_sp(2) * _q_trans_sp(2) + _q_trans_sp(3) * _q_trans_sp(3), -1.f, 1.f);
	const float tilt = acosf(cos_tilt);

	if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1 && _dive_transition_active) {
		updateDiveTransition();

	} else if (_vtol_mode == vtol_mode::TRANSITION_FRONT_ABORT && _dive_transition_active) {
		updateDiveTransition();

	} else if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {

		// calculate pitching rate - and constrain to at least 0.1s transition time
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_f_trans_dur.get(), 0.1f);

		if (tilt < M_PI_2_F - math::radians(_param_fw_psp_off.get())) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       _time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}

	} else if (_vtol_mode == vtol_mode::TRANSITION_BACK) {

		// calculate pitching rate - and constrain to at least 0.1s transition time
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_b_trans_dur.get(), 0.1f);

		if (tilt > 0.01f) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       _time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}
	}

	if (!(_dive_transition_active && (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1
					  || _vtol_mode == vtol_mode::TRANSITION_FRONT_ABORT))) {
		_v_att_sp->thrust_body[2] = _mc_virtual_att_sp->thrust_body[2];
	}

	if (_vtol_mode == vtol_mode::TRANSITION_BACK) {
		const float progress = math::constrain(_time_since_trans_start / B_TRANS_THRUST_BLENDING_DURATION, 0.f, 1.f);
		blendThrottleBeginningBackTransition(progress);
	}

	_v_att_sp->timestamp = hrt_absolute_time();

	_q_trans_sp.copyTo(_v_att_sp->q_d);
}

void Tailsitter::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust_body[0] = -_last_thr_in_mc;
}

void Tailsitter::update_fw_state()
{
	VtolType::update_fw_state();

}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	// Motors
	if (_vtol_mode == vtol_mode::FW_MODE) {

		const float fw_thrust = -_vehicle_thrust_setpoint_virtual_fw->xyz[0];
		_thrust_setpoint_0->xyz[2] = fw_thrust;

		/* allow differential thrust if enabled */
		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::YAW_BIT)) {
			_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0] * _param_vt_fw_difthr_s_y.get();
		}

		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::PITCH_BIT)) {
			_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1] * _param_vt_fw_difthr_s_p.get();
		}

		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::ROLL_BIT)) {
			_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2] * _param_vt_fw_difthr_s_r.get();
		}

		// Preserve the stock 50 ms FW-controller startup hold when the
		// experimental handoff is disabled.
		if (!_dive_handoff_active && hrt_elapsed_time(&_trans_finished_ts) < 50_ms) {
			_thrust_setpoint_0->xyz[2] = _last_thr_in_mc;
			_torque_setpoint_0->xyz[0] = 0.f;
			_torque_setpoint_0->xyz[1] = 0.f;
			_torque_setpoint_0->xyz[2] = 0.f;

		} else if (_dive_handoff_active) {
			const float progress = math::constrain((float)hrt_elapsed_time(&_dive_handoff_start_ts) * 1e-6f, 0.f, 1.f);
			_thrust_setpoint_0->xyz[2] = (1.f - progress) * _dive_handoff_thrust + progress * fw_thrust;
			const Vector3f fw_torque(_torque_setpoint_0->xyz);
			const Vector3f blended_torque = (1.f - progress) * _dive_handoff_torque + progress * fw_torque;
			blended_torque.copyTo(_torque_setpoint_0->xyz);

			if (progress >= 1.f) {
				_dive_handoff_active = false;
			}
		}

	} else {
		_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2];

		// for the short period after starting the backtransition where there is no thrust published yet from the MC controller,
		// keep publishing the last FW thrust to keep the motors running
		if (_vtol_mode != vtol_mode::TRANSITION_FRONT_P1
		    && _vtol_mode != vtol_mode::TRANSITION_FRONT_ABORT
		    && hrt_elapsed_time(&_transition_start_timestamp) < 50_ms) {
			_thrust_setpoint_0->xyz[2] = -_last_thr_in_fw_mode;
		}

		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0];
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1];
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2];
	}

	// Control surfaces
	if (!_param_vt_elev_mc_lock.get() || _vtol_mode != vtol_mode::MC_MODE) {
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2];
	}
}


bool Tailsitter::isFrontTransitionCompletedBase()
{
	const bool airspeed_triggers_transition = PX4_ISFINITE(_attc->get_calibrated_airspeed());

	bool transition_to_fw = false;
	const float pitch = Eulerf(Quatf(_v_att->q)).theta();

	if (pitch <= PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW) {
		if (airspeed_triggers_transition) {
			transition_to_fw = _attc->get_calibrated_airspeed() >= _param_vt_arsp_trans.get() ;

		} else {
			transition_to_fw = true;
		}
	}

	return transition_to_fw;
}

void Tailsitter::blendThrottleAfterFrontTransition(float scale)
{
	// note: MC throttle is negative (as in negative z), while FW throttle is positive (positive x)
	_v_att_sp->thrust_body[0] = scale * _v_att_sp->thrust_body[0] + (1.f - scale) * (-_last_thr_in_mc);
}

void Tailsitter::blendThrottleBeginningBackTransition(float scale)
{
	_v_att_sp->thrust_body[2] = scale * _v_att_sp->thrust_body[2] + (1.f - scale) * (-_last_thr_in_fw_mode);
}
