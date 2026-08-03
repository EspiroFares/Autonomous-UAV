// Independent system-level watchdog. Observes vehicle state and the commands
// leaving setpoint_validation_node, and publishes a verdict on /safety/status
// at a fixed rate. It never publishes a setpoint - hold_failsafe_node is what
// acts on the verdict, so a bug in here cannot itself command the aircraft.
//
// All thresholds live in safety_checks.hpp. This file only measures durations.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "drone_interfaces/msg/control_setpoint.hpp"
#include "drone_interfaces/msg/safety_status.hpp"
#include "drone_interfaces/msg/vehicle_status.hpp"
#include "drone_control/setpoint_limits.hpp"
#include "drone_safety/safety_checks.hpp"

using namespace std::chrono_literals;

class SafetySupervisionNode : public rclcpp::Node {
public:
    SafetySupervisionNode()
    : Node("safety_supervision_node"),
      has_status_(false),
      has_height_(false),
      armed_(false),
      height_m_(0.0f),
      saturation_duration_s_(0.0f),
      last_veto_(false)
    {
        last_status_time_ = this->now();
        last_height_time_ = this->now();
        last_saturation_check_ = this->now();

        status_sub_ = this->create_subscription<drone_interfaces::msg::VehicleStatus>(
            "/vehicle/status", 10,
            std::bind(&SafetySupervisionNode::StatusCallback, this, std::placeholders::_1));

        height_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/vehicle/height", 10,
            std::bind(&SafetySupervisionNode::HeightCallback, this, std::placeholders::_1));

        setpoint_sub_ = this->create_subscription<drone_interfaces::msg::ControlSetpoint>(
            "/control/setpoint_validated", 10,
            std::bind(&SafetySupervisionNode::SetpointCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<drone_interfaces::msg::SafetyStatus>(
            "/safety/status", 10);

        // 10 Hz. hold_failsafe_node treats silence beyond 1 s as a veto, so
        // this is ten heartbeats of margin against scheduling jitter.
        timer_ = this->create_wall_timer(100ms, std::bind(&SafetySupervisionNode::Update, this));

        RCLCPP_INFO(this->get_logger(), "safety_supervision_node started");
    }

private:
    void StatusCallback(const drone_interfaces::msg::VehicleStatus::SharedPtr msg) {
        armed_ = msg->armed;
        last_status_time_ = this->now();
        has_status_ = true;
    }

    void HeightCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        height_m_ = msg->data;
        last_height_time_ = this->now();
        has_height_ = true;
    }

    // Saturation is measured against the same envelope setpoint_validation_node
    // applies, taken from its header so the two cannot drift apart.
    void SetpointCallback(const drone_interfaces::msg::ControlSetpoint::SharedPtr msg) {
        const rclcpp::Time now = this->now();
        const float dt = static_cast<float>((now - last_saturation_check_).seconds());
        last_saturation_check_ = now;

        if (AtEnvelope(*msg)) {
            // Ignore an implausible dt after a gap in the stream; the freshness
            // checks own that case.
            if (dt > 0.0f && dt < 1.0f) {
                saturation_duration_s_ += dt;
            }
        } else {
            saturation_duration_s_ = 0.0f;
        }
    }

    bool AtEnvelope(const drone_interfaces::msg::ControlSetpoint & cmd) const {
        const drone_control::VelocityLimits limits;
        const float margin = 1e-3f;
        return std::abs(cmd.vx) >= limits.max_vx - margin ||
               std::abs(cmd.vy) >= limits.max_vy - margin ||
               std::abs(cmd.vz) >= limits.max_vz - margin ||
               std::abs(cmd.yaw_rate) >= limits.max_yaw_rate - margin;
    }

    void Update() {
        drone_safety::SupervisorInputs in;
        in.has_status = has_status_;
        in.status_age_s = static_cast<float>((this->now() - last_status_time_).seconds());
        in.armed = armed_;
        in.has_height = has_height_;
        in.height_age_s = static_cast<float>((this->now() - last_height_time_).seconds());
        in.height_m = height_m_;
        in.saturation_duration_s = saturation_duration_s_;

        const auto verdict = drone_safety::Evaluate(in);

        drone_interfaces::msg::SafetyStatus msg;
        msg.stamp = this->now();
        msg.veto = verdict.veto;
        msg.code = verdict.code;
        msg.reason = verdict.reason;
        pub_->publish(msg);

        if (verdict.veto != last_veto_) {
            if (verdict.veto) {
                RCLCPP_WARN(this->get_logger(), "veto: %s", verdict.reason.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "veto cleared");
            }
            last_veto_ = verdict.veto;
        }
    }

    rclcpp::Subscription<drone_interfaces::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_;
    rclcpp::Subscription<drone_interfaces::msg::ControlSetpoint>::SharedPtr setpoint_sub_;
    rclcpp::Publisher<drone_interfaces::msg::SafetyStatus>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_status_time_;
    rclcpp::Time last_height_time_;
    rclcpp::Time last_saturation_check_;

    bool has_status_;
    bool has_height_;
    bool armed_;
    float height_m_;
    float saturation_duration_s_;
    bool last_veto_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetySupervisionNode>());
    rclcpp::shutdown();
    return 0;
}
