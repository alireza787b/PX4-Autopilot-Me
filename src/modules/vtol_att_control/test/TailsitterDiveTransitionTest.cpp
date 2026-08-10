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

#include <gtest/gtest.h>

#include "../TailsitterDiveTransition.hpp"

#include <mathlib/mathlib.h>

class TailsitterDiveTransitionTest : public ::testing::Test
{
protected:
	TailsitterDiveTransition::Config config()
	{
		TailsitterDiveTransition::Config config{};
		config.max_dive_angle = math::radians(10.f);
		config.transition_rate = math::radians(15.f);
		config.transition_acceleration = math::radians(30.f);
		config.abort_rate = math::radians(20.f);
		config.abort_acceleration = math::radians(40.f);
		config.handoff_tilt = M_PI_2_F;
		config.transition_airspeed = 25.f;
		config.minimum_transition_time = 2.f;
		config.recovery_margin = 20.f;
		config.recovery_acceleration = 2.f;
		config.attitude_error_limit = math::radians(35.f);
		config.allocation_error_limit = 0.05f;
		config.handoff_rate_limit = math::radians(30.f);
		return config;
	}

	TailsitterDiveTransition::Input nominalInput(float elapsed, float airspeed = 20.f)
	{
		TailsitterDiveTransition::Input input{};
		input.dt = 0.01f;
		input.elapsed = elapsed;
		input.airspeed = airspeed;
		input.actual_tilt = 0.f;
		input.attitude_error = 0.f;
		input.angular_rate = 0.f;
		input.available_recovery_height = 100.f;
		input.vertical_speed_down = 0.f;
		input.airspeed_valid = true;
		input.handoff_ready = true;
		return input;
	}
};

TEST_F(TailsitterDiveTransitionTest, ReachesButNeverExceedsMaximumDiveAngle)
{
	TailsitterDiveTransition transition;
	transition.start(config());

	for (int i = 0; i < 2000; ++i) {
		auto input = nominalInput(i * 0.01f);
		const auto output = transition.update(input);
		EXPECT_LE(output.tilt, M_PI_2_F + math::radians(10.f) + 1e-5f);
	}

	EXPECT_EQ(transition.phase(), TailsitterDiveTransition::Phase::Dive);
	EXPECT_NEAR(transition.tilt(), M_PI_2_F + math::radians(10.f), 1e-4f);
}

TEST_F(TailsitterDiveTransitionTest, AirspeedBeforeHorizonAvoidsDive)
{
	TailsitterDiveTransition transition;
	transition.start(config());
	float maximum_tilt = 0.f;

	for (int i = 0; i < 1200 && transition.phase() != TailsitterDiveTransition::Phase::Complete; ++i) {
		auto input = nominalInput(i * 0.01f, 26.f);
		const auto output = transition.update(input);
		maximum_tilt = math::max(maximum_tilt, output.tilt);
	}

	EXPECT_EQ(transition.phase(), TailsitterDiveTransition::Phase::Complete);
	EXPECT_LE(maximum_tilt, M_PI_2_F + 1e-4f);
}

TEST_F(TailsitterDiveTransitionTest, AirspeedDuringDiveStartsCapture)
{
	TailsitterDiveTransition transition;
	transition.start(config());
	float elapsed = 0.f;

	while (transition.tilt() < M_PI_2_F + math::radians(5.f)) {
		transition.update(nominalInput(elapsed));
		elapsed += 0.01f;
	}

	for (int i = 0; i < 40; ++i) {
		transition.update(nominalInput(elapsed, 26.f));
		elapsed += 0.01f;
	}

	EXPECT_EQ(transition.phase(), TailsitterDiveTransition::Phase::Capture);

	while (transition.phase() != TailsitterDiveTransition::Phase::Complete && elapsed < 30.f) {
		transition.update(nominalInput(elapsed, 26.f));
		elapsed += 0.01f;
	}

	EXPECT_EQ(transition.phase(), TailsitterDiveTransition::Phase::Complete);
	EXPECT_NEAR(transition.tilt(), M_PI_2_F, 1e-3f);
}

TEST_F(TailsitterDiveTransitionTest, AirspeedDwellAndHysteresisRejectChatter)
{
	TailsitterDiveTransition transition;
	transition.start(config());

	for (int i = 0; i < 20; ++i) {
		transition.update(nominalInput(i * 0.01f, 26.f));
	}

	EXPECT_EQ(transition.phase(), TailsitterDiveTransition::Phase::Rotate);

	for (int i = 20; i < 35; ++i) {
		transition.update(nominalInput(i * 0.01f, 26.f));
	}

	EXPECT_EQ(transition.phase(), TailsitterDiveTransition::Phase::Capture);

	const auto output = transition.update(nominalInput(0.36f, 23.5f));
	EXPECT_TRUE(output.request_abort);
	EXPECT_EQ(output.abort_reason, TailsitterDiveTransition::AbortReason::AirspeedInvalid);
}

TEST_F(TailsitterDiveTransitionTest, InvalidAirspeedFailsClosed)
{
	TailsitterDiveTransition transition;
	transition.start(config());
	auto input = nominalInput(0.f);
	input.airspeed_valid = false;

	const auto output = transition.update(input);
	EXPECT_TRUE(output.request_abort);
	EXPECT_EQ(output.abort_reason, TailsitterDiveTransition::AbortReason::AirspeedInvalid);
}

TEST_F(TailsitterDiveTransitionTest, WaitsForFreshFixedWingSetpointBeforeHandoff)
{
	TailsitterDiveTransition transition;
	transition.start(config());
	float elapsed = 0.f;

	while (elapsed < 15.f) {
		auto input = nominalInput(elapsed, 26.f);
		input.handoff_ready = false;
		transition.update(input);
		elapsed += input.dt;
	}

	EXPECT_EQ(transition.phase(), TailsitterDiveTransition::Phase::Capture);

	while (transition.phase() != TailsitterDiveTransition::Phase::Complete && elapsed < 20.f) {
		transition.update(nominalInput(elapsed, 26.f));
		elapsed += 0.01f;
	}

	EXPECT_EQ(transition.phase(), TailsitterDiveTransition::Phase::Complete);
}

TEST_F(TailsitterDiveTransitionTest, DynamicRecoveryHeightIncludesRateAndStoppingDistance)
{
	const float height = TailsitterDiveTransition::requiredRecoveryHeight(math::radians(100.f), 8.f,
			     math::radians(20.f), 20.f, 2.f);
	EXPECT_NEAR(height, 40.f, 1e-4f);
}

TEST_F(TailsitterDiveTransitionTest, RecoveryMarginRequestsAbort)
{
	TailsitterDiveTransition transition;
	transition.start(config(), math::radians(100.f));
	auto input = nominalInput(5.f);
	input.actual_tilt = math::radians(100.f);
	input.vertical_speed_down = 8.f;
	input.available_recovery_height = 39.f;

	const auto output = transition.update(input);
	EXPECT_TRUE(output.request_abort);
	EXPECT_EQ(output.abort_reason, TailsitterDiveTransition::AbortReason::RecoveryAltitude);
}

TEST_F(TailsitterDiveTransitionTest, PersistentTrackingAndAllocationErrorsAbort)
{
	TailsitterDiveTransition transition;
	transition.start(config());
	auto input = nominalInput(0.f);
	input.attitude_error = math::radians(40.f);

	TailsitterDiveTransition::Output output{};

	for (int i = 0; i < 60; ++i) {
		output = transition.update(input);
	}

	EXPECT_TRUE(output.request_abort);
	EXPECT_EQ(output.abort_reason, TailsitterDiveTransition::AbortReason::AttitudeTracking);

	transition.start(config());
	input = nominalInput(0.f);
	input.allocation_status_valid = true;
	input.unallocated_pitch_torque = 0.1f;

	for (int i = 0; i < 60; ++i) {
		output = transition.update(input);
	}

	EXPECT_TRUE(output.request_abort);
	EXPECT_EQ(output.abort_reason, TailsitterDiveTransition::AbortReason::ControlAllocation);
}

TEST_F(TailsitterDiveTransitionTest, AbortReturnsSmoothlyToHover)
{
	TailsitterDiveTransition transition;
	transition.start(config(), math::radians(100.f), math::radians(10.f));
	transition.startAbort(math::radians(100.f), math::radians(10.f),
			      TailsitterDiveTransition::AbortReason::External);

	float previous_rate = transition.tiltRate();

	for (int i = 0; i < 1500 && transition.phase() != TailsitterDiveTransition::Phase::Recovered; ++i) {
		auto input = nominalInput(i * 0.01f);
		const auto output = transition.update(input);
		EXPECT_LE(fabsf(output.tilt_rate - previous_rate), math::radians(40.f) * 0.01f + 1e-5f);
		previous_rate = output.tilt_rate;
	}

	EXPECT_EQ(transition.phase(), TailsitterDiveTransition::Phase::Recovered);
	EXPECT_NEAR(transition.tilt(), 0.f, 1e-4f);
}
