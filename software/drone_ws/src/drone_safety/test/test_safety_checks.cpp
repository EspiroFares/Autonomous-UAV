// Unit tests for the supervisor checks. Two things matter here: that a fault
// produces a veto with a reason worth reading in a flight log, and that a
// disarmed aircraft on the floor is not vetoed for sitting there.

#include <string>

#include <gtest/gtest.h>

#include "drone_safety/safety_checks.hpp"

using drone_interfaces::msg::SafetyStatus;
using drone_safety::Evaluate;
using drone_safety::SafetyThresholds;
using drone_safety::SupervisorInputs;

namespace
{

// Armed, airborne at the target altitude, everything arriving on time.
SupervisorInputs HealthyFlight()
{
    SupervisorInputs in;
    in.has_status = true;
    in.status_age_s = 0.1f;
    in.armed = true;
    in.has_height = true;
    in.height_age_s = 0.1f;
    in.height_m = 1.2f;
    in.saturation_duration_s = 0.0f;
    return in;
}

}  // namespace

TEST(SafetyChecks, HealthyFlightIsNotVetoed)
{
    const auto out = Evaluate(HealthyFlight());

    EXPECT_FALSE(out.veto);
    EXPECT_EQ(out.code, SafetyStatus::CODE_OK);
}

TEST(SafetyChecks, VetoesWhenVehicleStatusIsMissing)
{
    auto in = HealthyFlight();
    in.has_status = false;

    const auto out = Evaluate(in);

    EXPECT_TRUE(out.veto);
    EXPECT_EQ(out.code, SafetyStatus::CODE_INPUT_SILENT);
}

TEST(SafetyChecks, VetoesWhenHeightGoesSilent)
{
    auto in = HealthyFlight();
    in.height_age_s = 3.0f;

    const auto out = Evaluate(in);

    EXPECT_TRUE(out.veto);
    EXPECT_EQ(out.code, SafetyStatus::CODE_INPUT_SILENT);
}

// The rangefinder dies after every FC reboot until the stream is re-requested.
// A silent height topic means altitude hold is running blind.
TEST(SafetyChecks, SilenceIsVetoedEvenWhenDisarmed)
{
    auto in = HealthyFlight();
    in.armed = false;
    in.has_height = false;

    EXPECT_TRUE(Evaluate(in).veto);
}

TEST(SafetyChecks, DisarmedOnTheFloorIsNotVetoed)
{
    auto in = HealthyFlight();
    in.armed = false;
    in.height_m = 0.05f;

    const auto out = Evaluate(in);

    EXPECT_FALSE(out.veto);
    EXPECT_EQ(out.code, SafetyStatus::CODE_OK);
}

TEST(SafetyChecks, VetoesAboveTheAltitudeBand)
{
    auto in = HealthyFlight();
    in.height_m = 2.8f;

    const auto out = Evaluate(in);

    EXPECT_TRUE(out.veto);
    EXPECT_EQ(out.code, SafetyStatus::CODE_ALTITUDE_OUT_OF_BAND);
    EXPECT_NE(out.reason.find("2.80"), std::string::npos);
    EXPECT_NE(out.reason.find("above"), std::string::npos);
}

TEST(SafetyChecks, VetoesBelowTheAltitudeBand)
{
    auto in = HealthyFlight();
    in.height_m = 0.1f;

    const auto out = Evaluate(in);

    EXPECT_TRUE(out.veto);
    EXPECT_EQ(out.code, SafetyStatus::CODE_ALTITUDE_OUT_OF_BAND);
    EXPECT_NE(out.reason.find("below"), std::string::npos);
}

// Climbing through the low end during takeoff must not trip the band.
TEST(SafetyChecks, AcceptsTheWholeBandInclusive)
{
    const SafetyThresholds limits;
    auto in = HealthyFlight();

    in.height_m = limits.min_altitude_m;
    EXPECT_FALSE(Evaluate(in, limits).veto);

    in.height_m = limits.max_altitude_m;
    EXPECT_FALSE(Evaluate(in, limits).veto);
}

// Guards against gains tuned past the safety envelope from the dashboard: the
// controller then demands more than it is allowed and stays pinned there.
TEST(SafetyChecks, VetoesASetpointSaturatedTooLong)
{
    auto in = HealthyFlight();
    in.saturation_duration_s = 5.0f;

    const auto out = Evaluate(in);

    EXPECT_TRUE(out.veto);
    EXPECT_EQ(out.code, SafetyStatus::CODE_SETPOINT_SATURATED);
}

// Brief saturation is normal when accelerating away from a standstill.
TEST(SafetyChecks, ToleratesShortSaturation)
{
    auto in = HealthyFlight();
    in.saturation_duration_s = 1.0f;

    EXPECT_FALSE(Evaluate(in).veto);
}

TEST(SafetyChecks, SilenceOutranksAnAltitudeFault)
{
    auto in = HealthyFlight();
    in.has_status = false;
    in.height_m = 9.0f;

    EXPECT_EQ(Evaluate(in).code, SafetyStatus::CODE_INPUT_SILENT);
}

TEST(SafetyChecks, RespectsATighterBand)
{
    SafetyThresholds indoors;
    indoors.max_altitude_m = 1.5f;

    auto in = HealthyFlight();
    in.height_m = 2.0f;

    EXPECT_TRUE(Evaluate(in, indoors).veto);
}
