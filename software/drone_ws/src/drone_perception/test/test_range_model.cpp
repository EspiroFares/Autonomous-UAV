// Unit tests for the monocular range maths in target_estimator_node.
//
// These cover the two failures that actually bit during flight testing: a
// single bad frame dragging the range estimate, and a spurious range spike
// being taken at face value and sent to the controller as "the person moved".

#include <cmath>
#include <deque>
#include <limits>

#include <gtest/gtest.h>

#include "drone_perception/range_model.hpp"

using drone_perception::IsImpossibleJump;
using drone_perception::IsRangePlausible;
using drone_perception::MotionGate;
using drone_perception::PushMedian;
using drone_perception::RangeFromPixels;

namespace
{
// The calibration actually flown, from drone_bringup/config/real_perception.yaml.
constexpr float kShoulderScale = 405.014f;
constexpr float kShoulderOffset = -0.314f;
}  // namespace

// ---------------------------------------------------------------- range model

TEST(RangeModel, MatchesTheGroundCalibrationAtAKnownWidth)
{
    // 405.014 / 100 - 0.314
    EXPECT_NEAR(RangeFromPixels(100.0f, kShoulderScale, kShoulderOffset), 3.736f, 1e-3f);
}

TEST(RangeModel, MoreShoulderPixelsMeansCloser)
{
    const float near = RangeFromPixels(200.0f, kShoulderScale, kShoulderOffset);
    const float far = RangeFromPixels(80.0f, kShoulderScale, kShoulderOffset);

    EXPECT_LT(near, far);
}

// The epsilon in the denominator exists for exactly this case. An infinity here
// would propagate through every downstream comparison as false and silently
// defeat the plausibility check.
TEST(RangeModel, ZeroPixelsStaysFiniteInsteadOfDividingByZero)
{
    const float range = RangeFromPixels(0.0f, kShoulderScale, kShoulderOffset);

    EXPECT_TRUE(std::isfinite(range));
    EXPECT_FALSE(IsRangePlausible(range, 0.3f, 5.0f));
}

TEST(RangeModel, PlausibilityRejectsRangesOutsideTheConfiguredBand)
{
    EXPECT_TRUE(IsRangePlausible(2.0f, 0.3f, 5.0f));
    EXPECT_FALSE(IsRangePlausible(0.1f, 0.3f, 5.0f));
    EXPECT_FALSE(IsRangePlausible(9.0f, 0.3f, 5.0f));
    EXPECT_FALSE(IsRangePlausible(std::numeric_limits<float>::quiet_NaN(), 0.3f, 5.0f));
}

// --------------------------------------------------------------- median window

TEST(MedianWindow, ASingleSampleIsItsOwnMedian)
{
    std::deque<float> window;
    EXPECT_FLOAT_EQ(PushMedian(window, 2.5f, 3), 2.5f);
}

TEST(MedianWindow, RejectsASingleOutlierFrame)
{
    std::deque<float> window;
    PushMedian(window, 2.0f, 3);
    PushMedian(window, 2.1f, 3);

    // One wild frame must not move the estimate.
    EXPECT_FLOAT_EQ(PushMedian(window, 9.0f, 3), 2.1f);
}

TEST(MedianWindow, EvenSizedWindowAveragesTheTwoMiddleSamples)
{
    std::deque<float> window;
    PushMedian(window, 1.0f, 2);

    EXPECT_FLOAT_EQ(PushMedian(window, 3.0f, 2), 2.0f);
}

TEST(MedianWindow, DropsTheOldestSampleOnceFull)
{
    std::deque<float> window;
    for (float sample : {1.0f, 1.0f, 1.0f}) {
        PushMedian(window, sample, 3);
    }
    ASSERT_EQ(window.size(), 3u);

    // Three new samples must fully displace the old ones.
    PushMedian(window, 5.0f, 3);
    PushMedian(window, 5.0f, 3);

    EXPECT_FLOAT_EQ(PushMedian(window, 5.0f, 3), 5.0f);
    EXPECT_EQ(window.size(), 3u);
}

// ----------------------------------------------------------------- physics gate

TEST(PhysicsGate, AcceptsOrdinaryWalkingSpeed)
{
    // 0.4 m over 200 ms is 2 m/s - a person walking briskly.
    EXPECT_FALSE(IsImpossibleJump(0.4f, 0.2f));
}

TEST(PhysicsGate, RejectsALargeStepAtImpossibleSpeed)
{
    // 2.5 m in one 200 ms frame implies 12.5 m/s. Nobody does that.
    EXPECT_TRUE(IsImpossibleJump(2.5f, 0.2f));
}

// The gate is a conjunction, and both halves are load-bearing. Testing them
// separately is the point: either one alone would reject legitimate motion.
TEST(PhysicsGate, ALargeStepAcrossALongGapIsJustWalking)
{
    // 1.5 m over 2 s is 0.75 m/s - large step, ordinary speed.
    EXPECT_FALSE(IsImpossibleJump(1.5f, 2.0f));
}

TEST(PhysicsGate, ASmallStepAtHighSpeedIsJustACorrection)
{
    // 0.5 m over 20 ms is 25 m/s, but the step itself is within noise.
    EXPECT_FALSE(IsImpossibleJump(0.5f, 0.02f));
}

TEST(PhysicsGate, NonPositiveTimestepIsNotTreatedAsAJump)
{
    // Guards the division. A zero or backwards dt means the caller has no
    // usable continuity, not that the target teleported.
    EXPECT_FALSE(IsImpossibleJump(5.0f, 0.0f));
    EXPECT_FALSE(IsImpossibleJump(5.0f, -0.1f));
}

TEST(PhysicsGate, RespectsATighterCustomGate)
{
    MotionGate strict;
    strict.max_jump_m = 0.2f;
    strict.max_rate_mps = 1.0f;

    EXPECT_TRUE(IsImpossibleJump(0.4f, 0.2f, strict));
}
