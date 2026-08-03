// Veto policy for hold_failsafe_node: decides whether a validated setpoint may
// reach the FC or whether the aircraft holds instead. Pure function of a
// snapshot (no clock, no ROS) so every branch is unit testable.

#ifndef DRONE_SAFETY__HOLD_POLICY_HPP_
#define DRONE_SAFETY__HOLD_POLICY_HPP_

#include <cstdint>

#include "drone_interfaces/msg/control_setpoint.hpp"

namespace drone_safety
{

enum class HoldReason : std::uint8_t
{
    kPassThrough = 0,
    kSupervisorSilent,
    kSupervisorVeto,
    kNoSetpointYet,
    kStaleSetpoint,
    kUpstreamRequestedHold,
};

struct HoldLimits
{
    // Same 0.5 s staleness rule the other timer-driven nodes use.
    float max_setpoint_age_s = 0.5f;
    // Supervisor publishes at 10 Hz, so this is ten missed heartbeats.
    float max_status_age_s = 1.0f;
};

// Ages are passed in rather than read from a clock, so a decision is
// reproducible in a test.
struct HoldInputs
{
    bool has_setpoint = false;
    float setpoint_age_s = 0.0f;
    drone_interfaces::msg::ControlSetpoint setpoint;

    bool has_status = false;
    float status_age_s = 0.0f;
    bool status_veto = false;
};

struct HoldDecision
{
    drone_interfaces::msg::ControlSetpoint command;
    HoldReason reason = HoldReason::kPassThrough;
    bool holding = false;
};

// Zeros, never the last good command — repeating a stale command keeps the
// drone moving on data nobody trusts any more.
inline drone_interfaces::msg::ControlSetpoint HoldCommand()
{
    drone_interfaces::msg::ControlSetpoint cmd;
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.vz = 0.0f;
    cmd.yaw_rate = 0.0f;
    cmd.hold = true;
    return cmd;
}

// Checks run in severity order and the first hit wins. A dead safety layer
// outranks a dead controller: catching the controller is what it exists for.
inline HoldDecision Decide(
    const HoldInputs & in,
    const HoldLimits & limits = HoldLimits())
{
    HoldDecision out;
    out.holding = true;
    out.command = HoldCommand();

    // Silence is not health. Reading "no veto received" as "no veto exists"
    // would drop the safety layer exactly when it has crashed.
    if (!in.has_status || in.status_age_s > limits.max_status_age_s) {
        out.reason = HoldReason::kSupervisorSilent;
        return out;
    }

    if (in.status_veto) {
        out.reason = HoldReason::kSupervisorVeto;
        return out;
    }

    // Nothing commanded yet; hold is the only safe default at startup.
    if (!in.has_setpoint) {
        out.reason = HoldReason::kNoSetpointYet;
        return out;
    }

    if (in.setpoint_age_s > limits.max_setpoint_age_s) {
        out.reason = HoldReason::kStaleSetpoint;
        return out;
    }

    // Controller asked to hold. Emit the canonical hold rather than forwarding
    // its own zeros, so downstream only ever sees one shape of hold.
    if (in.setpoint.hold) {
        out.reason = HoldReason::kUpstreamRequestedHold;
        return out;
    }

    out.command = in.setpoint;
    out.holding = false;
    out.reason = HoldReason::kPassThrough;
    return out;
}

}  // namespace drone_safety

#endif  // DRONE_SAFETY__HOLD_POLICY_HPP_
