// System-level checks behind safety_supervision_node. The node only observes
// and publishes a verdict; it never touches a setpoint. Pure function of a
// snapshot, same as hold_policy.hpp, so the thresholds are testable on the
// ground instead of in the air.

#ifndef DRONE_SAFETY__SAFETY_CHECKS_HPP_
#define DRONE_SAFETY__SAFETY_CHECKS_HPP_

#include <cstdint>
#include <cstdio>
#include <string>

#include "drone_interfaces/msg/safety_status.hpp"

namespace drone_safety
{

struct SafetyThresholds
{
    // Indoor envelope. The floor is below the ~0.3 m the lidar reads on the
    // ground so a disarmed aircraft is not permanently vetoed.
    float min_altitude_m = 0.2f;
    float max_altitude_m = 2.5f;

    // Only fires if the controller is asking for more than the safety envelope
    // allows, which needs someone to tune its gains past the envelope from the
    // dashboard mid-flight. At the shipped gains every axis clamps itself well
    // below, so this catches misconfiguration rather than normal saturation.
    float max_saturation_s = 3.0f;

    // Anything the supervisor watches must arrive at least this often.
    float max_input_age_s = 1.0f;
};

struct SupervisorInputs
{
    bool has_status = false;
    float status_age_s = 0.0f;
    bool armed = false;

    bool has_height = false;
    float height_age_s = 0.0f;
    float height_m = 0.0f;

    // How long any setpoint axis has been sitting at its clamp, measured by
    // the node. Zero when nothing is saturated.
    float saturation_duration_s = 0.0f;
};

struct SafetyVerdict
{
    bool veto = false;
    std::uint8_t code = drone_interfaces::msg::SafetyStatus::CODE_OK;
    std::string reason;
};

inline std::string FormatMetres(float value)
{
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.2f", static_cast<double>(value));
    return std::string(buffer);
}

// Silence is checked first and applies whether or not the aircraft is armed:
// a supervisor that cannot see the vehicle has nothing to supervise. The
// altitude and saturation checks only apply in the air, since neither means
// anything while the drone sits on the floor.
inline SafetyVerdict Evaluate(
    const SupervisorInputs & in,
    const SafetyThresholds & limits = SafetyThresholds())
{
    using drone_interfaces::msg::SafetyStatus;

    SafetyVerdict out;

    if (!in.has_status || in.status_age_s > limits.max_input_age_s) {
        out.veto = true;
        out.code = SafetyStatus::CODE_INPUT_SILENT;
        out.reason = "no vehicle status";
        return out;
    }

    if (!in.has_height || in.height_age_s > limits.max_input_age_s) {
        out.veto = true;
        out.code = SafetyStatus::CODE_INPUT_SILENT;
        out.reason = "no height; altitude hold is blind";
        return out;
    }

    if (!in.armed) {
        return out;
    }

    if (in.height_m < limits.min_altitude_m) {
        out.veto = true;
        out.code = SafetyStatus::CODE_ALTITUDE_OUT_OF_BAND;
        out.reason = "altitude " + FormatMetres(in.height_m) + " m below band";
        return out;
    }

    if (in.height_m > limits.max_altitude_m) {
        out.veto = true;
        out.code = SafetyStatus::CODE_ALTITUDE_OUT_OF_BAND;
        out.reason = "altitude " + FormatMetres(in.height_m) + " m above band";
        return out;
    }

    if (in.saturation_duration_s > limits.max_saturation_s) {
        out.veto = true;
        out.code = SafetyStatus::CODE_SETPOINT_SATURATED;
        out.reason = "setpoint saturated for " +
            FormatMetres(in.saturation_duration_s) + " s";
        return out;
    }

    return out;
}

}  // namespace drone_safety

#endif  // DRONE_SAFETY__SAFETY_CHECKS_HPP_
