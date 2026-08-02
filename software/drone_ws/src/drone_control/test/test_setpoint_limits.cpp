// Unit tests for the velocity envelope enforced by setpoint_validation_node.
// This is the last thing that runs before a command reaches the flight
// controller, so the cases below are the ones that would actually hurt:
// a command outside the envelope, and a command that is not a number at all.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "drone_control/setpoint_limits.hpp"

using drone_control::ValidateSetpoint;
using drone_control::VelocityLimits;
using drone_interfaces::msg::ControlSetpoint;

namespace
{

ControlSetpoint MakeSetpoint(float vx, float vy, float vz, float yaw_rate)
{
    ControlSetpoint msg;
    msg.vx = vx;
    msg.vy = vy;
    msg.vz = vz;
    msg.yaw_rate = yaw_rate;
    msg.hold = false;
    return msg;
}

}  // namespace

TEST(SetpointLimits, PassesThroughCommandsInsideTheEnvelope)
{
    const auto out = ValidateSetpoint(MakeSetpoint(0.4f, -0.2f, 0.1f, 0.8f));

    EXPECT_FLOAT_EQ(out.vx, 0.4f);
    EXPECT_FLOAT_EQ(out.vy, -0.2f);
    EXPECT_FLOAT_EQ(out.vz, 0.1f);
    EXPECT_FLOAT_EQ(out.yaw_rate, 0.8f);
    EXPECT_FALSE(out.hold);
}

TEST(SetpointLimits, ClampsEachAxisToItsOwnLimit)
{
    const VelocityLimits limits;
    const auto out = ValidateSetpoint(MakeSetpoint(9.0f, 9.0f, 9.0f, 9.0f));

    EXPECT_FLOAT_EQ(out.vx, limits.max_vx);
    EXPECT_FLOAT_EQ(out.vy, limits.max_vy);
    EXPECT_FLOAT_EQ(out.vz, limits.max_vz);
    EXPECT_FLOAT_EQ(out.yaw_rate, limits.max_yaw_rate);
}

TEST(SetpointLimits, ClampsNegativeCommandsSymmetrically)
{
    const VelocityLimits limits;
    const auto out = ValidateSetpoint(MakeSetpoint(-9.0f, -9.0f, -9.0f, -9.0f));

    EXPECT_FLOAT_EQ(out.vx, -limits.max_vx);
    EXPECT_FLOAT_EQ(out.vy, -limits.max_vy);
    EXPECT_FLOAT_EQ(out.vz, -limits.max_vz);
    EXPECT_FLOAT_EQ(out.yaw_rate, -limits.max_yaw_rate);
}

// Climb rate is deliberately the tightest axis - a runaway vz is the one that
// puts the aircraft into the ceiling.
TEST(SetpointLimits, VerticalEnvelopeIsTighterThanHorizontal)
{
    const VelocityLimits limits;
    EXPECT_LT(limits.max_vz, limits.max_vx);
}

TEST(SetpointLimits, NanOnAnyAxisForcesAZeroedHold)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();

    for (int axis = 0; axis < 4; ++axis) {
        auto msg = MakeSetpoint(0.5f, 0.5f, 0.2f, 0.5f);
        switch (axis) {
            case 0: msg.vx = nan; break;
            case 1: msg.vy = nan; break;
            case 2: msg.vz = nan; break;
            default: msg.yaw_rate = nan; break;
        }

        const auto out = ValidateSetpoint(msg);

        EXPECT_TRUE(out.hold) << "axis " << axis;
        EXPECT_FLOAT_EQ(out.vx, 0.0f) << "axis " << axis;
        EXPECT_FLOAT_EQ(out.vy, 0.0f) << "axis " << axis;
        EXPECT_FLOAT_EQ(out.vz, 0.0f) << "axis " << axis;
        EXPECT_FLOAT_EQ(out.yaw_rate, 0.0f) << "axis " << axis;
    }
}

TEST(SetpointLimits, InfinityIsRejectedTheSameWayAsNan)
{
    auto msg = MakeSetpoint(0.5f, 0.0f, 0.0f, 0.0f);
    msg.vx = std::numeric_limits<float>::infinity();

    const auto out = ValidateSetpoint(msg);

    EXPECT_TRUE(out.hold);
    EXPECT_FLOAT_EQ(out.vx, 0.0f);
}

// A bad command must never be sanitised into a plausible-looking one: the
// controller downstream has to be told it is holding, not handed zeros that
// read like a deliberate stop.
TEST(SetpointLimits, HoldIsSetEvenWhenTheCallerDidNotAskForIt)
{
    auto msg = MakeSetpoint(0.5f, 0.0f, 0.0f, 0.0f);
    msg.vz = std::numeric_limits<float>::quiet_NaN();
    msg.hold = false;

    EXPECT_TRUE(ValidateSetpoint(msg).hold);
}

TEST(SetpointLimits, HoldFlagIsPreservedForValidCommands)
{
    auto msg = MakeSetpoint(0.3f, 0.0f, 0.0f, 0.0f);
    msg.hold = true;

    const auto out = ValidateSetpoint(msg);

    EXPECT_TRUE(out.hold);
    EXPECT_FLOAT_EQ(out.vx, 0.3f);
}

TEST(SetpointLimits, RespectsATighterCustomEnvelope)
{
    VelocityLimits cautious;
    cautious.max_vx = 0.2f;

    const auto out = ValidateSetpoint(MakeSetpoint(1.0f, 0.0f, 0.0f, 0.0f), cautious);

    EXPECT_FLOAT_EQ(out.vx, 0.2f);
}
