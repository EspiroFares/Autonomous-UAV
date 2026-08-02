// The monocular range maths behind target_estimator_node, kept free of rclcpp
// so it can be unit tested. A wrong number here does not crash anything - it
// quietly tells the controller the person is somewhere they are not, which is
// how a drone ends up lunging at its target.

#ifndef DRONE_PERCEPTION__RANGE_MODEL_HPP_
#define DRONE_PERCEPTION__RANGE_MODEL_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <vector>

namespace drone_perception
{

// Inverse pinhole model, calibrated on the ground: an object of fixed real
// width subtends fewer pixels the further away it is, so distance goes as
// 1/pixels. scale and offset come from a least-squares fit.
//
// The epsilon keeps a zero-pixel measurement from dividing by zero. It yields
// an absurdly large but finite distance, which the plausibility check below
// then rejects - that is deliberate, an inf would poison every comparison.
inline float RangeFromPixels(float pixels, float scale, float offset)
{
    return scale / (pixels + 1e-6f) + offset;
}

inline bool IsRangePlausible(float range_m, float min_m, float max_m)
{
    return std::isfinite(range_m) && range_m >= min_m && range_m <= max_m;
}

// Sliding median over the last few frames. A mean would let a single bad frame
// drag the estimate; a median ignores it entirely.
inline float PushMedian(std::deque<float> & window, float sample, std::size_t max_size)
{
    window.push_back(sample);
    while (window.size() > max_size) {
        window.pop_front();
    }

    std::vector<float> sorted(window.begin(), window.end());
    std::sort(sorted.begin(), sorted.end());

    const std::size_t middle = sorted.size() / 2;
    if (sorted.size() % 2 == 0) {
        return 0.5f * (sorted[middle - 1] + sorted[middle]);
    }
    return sorted[middle];
}

// What a human being can actually do between two frames.
struct MotionGate
{
    float max_jump_m = 1.0f;
    float max_rate_mps = 4.0f;
};

// Rejects a range change only when it is BOTH a large step AND implies a speed
// no person reaches. The conjunction matters: a large step across a long gap
// between frames is ordinary walking, and a fast step across a short gap is a
// small correction. Only both together mean the estimator lied.
inline bool IsImpossibleJump(float jump_m, float dt_s, const MotionGate & gate = MotionGate())
{
    if (dt_s <= 0.0f) {
        return false;
    }
    return jump_m > gate.max_jump_m && (jump_m / dt_s) > gate.max_rate_mps;
}

}  // namespace drone_perception

#endif  // DRONE_PERCEPTION__RANGE_MODEL_HPP_
