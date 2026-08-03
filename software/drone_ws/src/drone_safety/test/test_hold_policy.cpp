// Unit tests for the hold_failsafe_node veto policy. The interesting cases are
// the ones where something is missing rather than wrong: a silent supervisor
// and a setpoint that stopped arriving both have to end in a hold.

#include <vector>

#include <gtest/gtest.h>

#include "drone_safety/hold_policy.hpp"

using drone_safety::Decide;
using drone_safety::HoldInputs;
using drone_safety::HoldLimits;
using drone_safety::HoldReason;

namespace
{

// A snapshot with nothing wrong, so each test can break one field at a time.
HoldInputs HealthyInputs()
{
    HoldInputs in;
    in.has_status = true;
    in.status_age_s = 0.05f;
    in.status_veto = false;
    in.has_setpoint = true;
    in.setpoint_age_s = 0.05f;
    in.setpoint.vx = 0.4f;
    in.setpoint.yaw_rate = 0.2f;
    in.setpoint.hold = false;
    return in;
}

}  // namespace

TEST(HoldPolicy, PassesAHealthySetpointThrough)
{
    const auto out = Decide(HealthyInputs());

    EXPECT_FALSE(out.holding);
    EXPECT_EQ(out.reason, HoldReason::kPassThrough);
    EXPECT_FLOAT_EQ(out.command.vx, 0.4f);
    EXPECT_FLOAT_EQ(out.command.yaw_rate, 0.2f);
    EXPECT_FALSE(out.command.hold);
}

TEST(HoldPolicy, HoldsWhenTheSupervisorHasNeverPublished)
{
    auto in = HealthyInputs();
    in.has_status = false;

    const auto out = Decide(in);

    EXPECT_TRUE(out.holding);
    EXPECT_EQ(out.reason, HoldReason::kSupervisorSilent);
}

TEST(HoldPolicy, HoldsWhenTheSupervisorGoesSilent)
{
    auto in = HealthyInputs();
    in.status_age_s = 2.0f;

    EXPECT_EQ(Decide(in).reason, HoldReason::kSupervisorSilent);
}

TEST(HoldPolicy, HoldsOnAnExplicitVeto)
{
    auto in = HealthyInputs();
    in.status_veto = true;

    const auto out = Decide(in);

    EXPECT_TRUE(out.holding);
    EXPECT_EQ(out.reason, HoldReason::kSupervisorVeto);
}

TEST(HoldPolicy, HoldsBeforeTheFirstSetpointArrives)
{
    auto in = HealthyInputs();
    in.has_setpoint = false;

    EXPECT_EQ(Decide(in).reason, HoldReason::kNoSetpointYet);
}

TEST(HoldPolicy, HoldsOnAStaleSetpoint)
{
    auto in = HealthyInputs();
    in.setpoint_age_s = 0.8f;

    EXPECT_EQ(Decide(in).reason, HoldReason::kStaleSetpoint);
}

// The controller zeroes its own command when it asks to hold, but the velocity
// fields are not guaranteed. Downstream sees one shape of hold either way.
TEST(HoldPolicy, ForwardsAnUpstreamHoldAsTheCanonicalHold)
{
    auto in = HealthyInputs();
    in.setpoint.hold = true;

    const auto out = Decide(in);

    EXPECT_TRUE(out.holding);
    EXPECT_EQ(out.reason, HoldReason::kUpstreamRequestedHold);
    EXPECT_FLOAT_EQ(out.command.vx, 0.0f);
}

TEST(HoldPolicy, EveryHoldPathEmitsTheSameZeroedCommand)
{
    std::vector<HoldInputs> faults;

    auto silent = HealthyInputs();
    silent.has_status = false;
    faults.push_back(silent);

    auto veto = HealthyInputs();
    veto.status_veto = true;
    faults.push_back(veto);

    auto missing = HealthyInputs();
    missing.has_setpoint = false;
    faults.push_back(missing);

    auto stale = HealthyInputs();
    stale.setpoint_age_s = 5.0f;
    faults.push_back(stale);

    for (const auto & in : faults) {
        const auto out = Decide(in);
        EXPECT_TRUE(out.holding);
        EXPECT_TRUE(out.command.hold);
        EXPECT_FLOAT_EQ(out.command.vx, 0.0f);
        EXPECT_FLOAT_EQ(out.command.vy, 0.0f);
        EXPECT_FLOAT_EQ(out.command.vz, 0.0f);
        EXPECT_FLOAT_EQ(out.command.yaw_rate, 0.0f);
    }
}

// Both faults at once must be reported as the safety layer failing, not as the
// symptom downstream of it.
TEST(HoldPolicy, SupervisorSilenceOutranksAStaleSetpoint)
{
    auto in = HealthyInputs();
    in.has_status = false;
    in.setpoint_age_s = 5.0f;

    EXPECT_EQ(Decide(in).reason, HoldReason::kSupervisorSilent);
}

// Age checks use >, so a sample exactly at the limit still counts as fresh.
TEST(HoldPolicy, AcceptsAnAgeExactlyAtTheLimit)
{
    const HoldLimits limits;
    auto in = HealthyInputs();
    in.setpoint_age_s = limits.max_setpoint_age_s;
    in.status_age_s = limits.max_status_age_s;

    EXPECT_EQ(Decide(in, limits).reason, HoldReason::kPassThrough);
}

TEST(HoldPolicy, RespectsATighterStalenessLimit)
{
    HoldLimits strict;
    strict.max_setpoint_age_s = 0.02f;

    auto in = HealthyInputs();
    in.setpoint_age_s = 0.05f;

    EXPECT_EQ(Decide(in, strict).reason, HoldReason::kStaleSetpoint);
}
