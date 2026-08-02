// Pure velocity-envelope policy, kept free of rclcpp so it can be unit tested.
// setpoint_validation_node is the last gate before fcu_bridge_node hands a
// command to the flight controller, so this is the code that must not be wrong.

#ifndef DRONE_CONTROL__SETPOINT_LIMITS_HPP_
#define DRONE_CONTROL__SETPOINT_LIMITS_HPP_

#include <algorithm>
#include <cmath>

#include "drone_interfaces/msg/control_setpoint.hpp"

namespace drone_control
{

// Hard envelope the aircraft may never be commanded outside of.
struct VelocityLimits
{
    float max_vx = 1.0f;
    float max_vy = 1.0f;
    float max_vz = 0.5f;
    float max_yaw_rate = 1.5f;
};

// Any non-finite axis makes the whole command untrustworthy, so the result is
// a zeroed hold rather than a partially sanitised command. Otherwise each axis
// is clamped independently into the envelope.
//
// Note: stamp is deliberately not copied. fcu_bridge_node judges freshness by
// arrival time, not by this field.
inline drone_interfaces::msg::ControlSetpoint ValidateSetpoint(
    const drone_interfaces::msg::ControlSetpoint & in,
    const VelocityLimits & limits = VelocityLimits())
{
    drone_interfaces::msg::ControlSetpoint out;

    if (!std::isfinite(in.vx) || !std::isfinite(in.vy) ||
        !std::isfinite(in.vz) || !std::isfinite(in.yaw_rate))
    {
        out.vx = 0.0f;
        out.vy = 0.0f;
        out.vz = 0.0f;
        out.yaw_rate = 0.0f;
        out.hold = true;
        return out;
    }

    out.vx = std::clamp(in.vx, -limits.max_vx, limits.max_vx);
    out.vy = std::clamp(in.vy, -limits.max_vy, limits.max_vy);
    out.vz = std::clamp(in.vz, -limits.max_vz, limits.max_vz);
    out.yaw_rate = std::clamp(in.yaw_rate, -limits.max_yaw_rate, limits.max_yaw_rate);
    out.hold = in.hold;

    return out;
}

}  // namespace drone_control

#endif  // DRONE_CONTROL__SETPOINT_LIMITS_HPP_
