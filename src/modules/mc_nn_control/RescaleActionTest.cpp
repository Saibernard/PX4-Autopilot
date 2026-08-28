/****************************************************************************
*
*   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file RescaleActionTest.cpp
 * Characterization tests for the network action to motor command mapping,
 * including the out of range region reported in
 * https://github.com/PX4/PX4-Autopilot/issues/28417.
 *
 * to run: make tests TESTFILTER=RescaleAction
 */

#include <gtest/gtest.h>
#include <cmath>

#include "actions_rescale.hpp"

using nn_control::rescale_action;

// defaults from mc_nn_control_params.yaml
static constexpr float kThrustCoeff = 1.2f;
static constexpr float kMinRpm = 1000.f;
static constexpr float kMaxRpm = 22000.f;

static float map(float action)
{
	return rescale_action(action, kThrustCoeff, kMinRpm, kMaxRpm);
}

TEST(RescaleActionTest, actionsBeyondOneClampToTheBoundary)
{
	EXPECT_FLOAT_EQ(map(-5.f), map(-1.f));
	EXPECT_FLOAT_EQ(map(5.f), map(1.f));
}

TEST(RescaleActionTest, mappingIsMonotonic)
{
	float prev = map(-1.f);

	for (float action = -0.99f; action <= 1.f; action += 0.01f) {
		const float cmd = map(action);
		EXPECT_GT(cmd, prev) << "not monotonic at action " << action;
		prev = cmd;
	}
}

TEST(RescaleActionTest, lowestActionCommandsBelowZero)
{
	// issue 28417: with default parameters the bottom of the action range maps
	// below zero, which FunctionMotors turns into NaN and treats as a stopped
	// motor. This pins the current behaviour so a fix shows up as a diff here.
	EXPECT_NEAR(map(-1.f), -0.0077f, 5e-4f);
	EXPECT_LT(map(-1.f), 0.f);
}

TEST(RescaleActionTest, zeroCrossingSitsNearTheBottomOfTheRange)
{
	// the usable range starts where the command becomes non negative
	EXPECT_LT(map(-0.9975f), 0.f);
	EXPECT_GT(map(-0.9950f), 0.f);
}

TEST(RescaleActionTest, fullScaleIsReachedWellBelowActionOne)
{
	// issue 28417: command 1.0 is already produced around action 0.61, the rest
	// of the positive action range commands beyond full scale
	EXPECT_LT(map(0.60f), 1.f);
	EXPECT_GT(map(0.62f), 1.f);
	EXPECT_GT(map(1.f), 1.2f);
}

TEST(RescaleActionTest, nanActionPropagates)
{
	// the module relies on PublishOutput passing non finite values through and
	// on the downstream consumer to handle them
	EXPECT_FALSE(std::isfinite(map(NAN)));
}

TEST(RescaleActionTest, degenerateParametersProduceNonFiniteCommands)
{
	// nothing validates the parameters: equal rpm limits or a zero thrust
	// coefficient divide by zero
	EXPECT_FALSE(std::isfinite(rescale_action(0.f, kThrustCoeff, 5000.f, 5000.f)));
	EXPECT_FALSE(std::isfinite(rescale_action(0.f, 0.f, kMinRpm, kMaxRpm)));
}
