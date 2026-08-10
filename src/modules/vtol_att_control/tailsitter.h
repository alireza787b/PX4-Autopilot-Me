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
* @file tailsitter.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#ifndef TAILSITTER_H
#define TAILSITTER_H

#include "vtol_type.h"

#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>

#include "TailsitterDiveTransition.hpp"

// [rad] Pitch threshold required for completing transition to fixed-wing in automatic transitions
static constexpr float PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW = -1.05f; // -60°

// [rad] Pitch threshold required for completing transition to hover in automatic transitions
static constexpr float PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC = -0.26f; // -15°

// [s] Thrust blending duration from fixed-wing to back transition throttle
static constexpr float B_TRANS_THRUST_BLENDING_DURATION = 0.5f;

class Tailsitter : public VtolType
{

public:
	Tailsitter(VtolAttitudeControl *_att_controller);
	~Tailsitter() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;
	void blendThrottleBeginningBackTransition(float scale);

private:
	enum class vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_FRONT_ABORT,	/**< controlled abort of an opt-in front transition */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
		FW_MODE					/**< vtol is in fixed wing mode */
	};

	vtol_mode _vtol_mode{vtol_mode::MC_MODE};			/**< vtol flight mode, defined by enum vtol_mode */

	bool _flag_was_in_trans_mode = false;	// true if mode has just switched to transition

	matrix::Quatf _q_trans_start;
	matrix::Quatf _q_trans_sp;
	matrix::Vector3f _trans_rot_axis;

	TailsitterDiveTransition _dive_transition;
	TailsitterDiveTransition::Config _dive_transition_config{};
	bool _dive_transition_active{false};
	bool _dive_transition_initialized{false};
	bool _dive_handoff_active{false};
	hrt_abstime _dive_handoff_start_ts{0};
	float _dive_transition_thrust{0.f};
	float _dive_handoff_thrust{0.f};
	matrix::Vector3f _dive_handoff_torque{};

	inline static const matrix::Quatf _q_fw_to_mc{matrix::Eulerf{0.f, M_PI_2_F, 0.f}};

	void parameters_update() override;

	bool isFrontTransitionCompletedBase() override;
	bool isPitchExceeded() override;
	bool isRollExceeded() override;

	bool diveTransitionEnabled() const;
	void configureDiveTransition();
	void initializeControlledAbort(TailsitterDiveTransition::AbortReason reason);
	void updateDiveTransition();
	void updateDiveTransitionThrust(float commanded_tilt);
	float actualTransitionTilt() const;
	float actualTransitionTiltRate() const;
	float transitionAttitudeError() const;
	float availableRecoveryHeight() const;
	void triggerDiveAbort(TailsitterDiveTransition::AbortReason reason, bool quadchute);

	matrix::Eulerf getFixedWingAttitudeEuler() const;

	DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
					(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,
					(ParamBool<px4::params::VT_TS_DIVE_EN>) _param_vt_ts_dive_en,
					(ParamFloat<px4::params::VT_TS_DIVE_ANG>) _param_vt_ts_dive_ang,
					(ParamFloat<px4::params::VT_TS_DIVE_RAT>) _param_vt_ts_dive_rat,
					(ParamFloat<px4::params::VT_TS_DIVE_THR>) _param_vt_ts_dive_thr,
					(ParamFloat<px4::params::VT_TS_REC_ALT>) _param_vt_ts_rec_alt,
					(ParamFloat<px4::params::VT_TS_REC_ACC>) _param_vt_ts_rec_acc,
					(ParamFloat<px4::params::VT_TS_ABRT_RAT>) _param_vt_ts_abrt_rat,
					(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover
				       )


};
#endif
