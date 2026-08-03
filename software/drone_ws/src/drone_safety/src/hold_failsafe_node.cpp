// Pass-through veto between setpoint_validation_node and fcu_bridge_node.
// Forwards a validated setpoint only while the supervisor says it is safe and
// the command is fresh; otherwise publishes a hold. Timer-driven so it keeps
// emitting a command of its own after everything upstream has gone quiet.
//
// All the logic lives in hold_policy.hpp. This file only measures ages.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "drone_interfaces/msg/control_setpoint.hpp"
#include "drone_interfaces/msg/safety_status.hpp"
#include "drone_safety/hold_policy.hpp"

using namespace std::chrono_literals;

class HoldFailsafeNode : public rclcpp::Node {
public:
    HoldFailsafeNode()
    : Node("hold_failsafe_node"),
      has_setpoint_(false),
      has_status_(false),
      last_reason_(drone_safety::HoldReason::kNoSetpointYet)
    {
        // Seeded so the first Update() never subtracts from an uninitialised
        // Time; the has_ flags are what actually gate their use.
        last_setpoint_time_ = this->now();
        last_status_time_ = this->now();

        setpoint_sub_ = this->create_subscription<drone_interfaces::msg::ControlSetpoint>(
            "/control/setpoint_validated", 10,
            std::bind(&HoldFailsafeNode::SetpointCallback, this, std::placeholders::_1));

        status_sub_ = this->create_subscription<drone_interfaces::msg::SafetyStatus>(
            "/safety/status", 10,
            std::bind(&HoldFailsafeNode::StatusCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<drone_interfaces::msg::ControlSetpoint>(
            "/control/setpoint_safe", 10);

        // 20 Hz, matching follow_controller_node so a pass-through adds at
        // most one tick of latency.
        timer_ = this->create_wall_timer(50ms, std::bind(&HoldFailsafeNode::Update, this));

        RCLCPP_INFO(this->get_logger(), "hold_failsafe_node started");
    }

private:
    void SetpointCallback(const drone_interfaces::msg::ControlSetpoint::SharedPtr msg) {
        last_setpoint_ = *msg;
        last_setpoint_time_ = this->now();
        has_setpoint_ = true;
    }

    void StatusCallback(const drone_interfaces::msg::SafetyStatus::SharedPtr msg) {
        last_status_veto_ = msg->veto;
        last_status_reason_ = msg->reason;
        last_status_time_ = this->now();
        has_status_ = true;
    }

    void Update() {
        drone_safety::HoldInputs in;
        in.has_setpoint = has_setpoint_;
        in.setpoint = last_setpoint_;
        in.setpoint_age_s = static_cast<float>((this->now() - last_setpoint_time_).seconds());
        in.has_status = has_status_;
        in.status_veto = last_status_veto_;
        in.status_age_s = static_cast<float>((this->now() - last_status_time_).seconds());

        const auto decision = drone_safety::Decide(in);

        auto out = decision.command;
        out.stamp = this->now();
        pub_->publish(out);

        // Only on change: at 20 Hz a per-tick log would bury the flight log.
        if (decision.reason != last_reason_) {
            if (!decision.holding) {
                RCLCPP_INFO(this->get_logger(), "passing setpoints through");
            } else if (decision.reason == drone_safety::HoldReason::kSupervisorVeto) {
                RCLCPP_WARN(this->get_logger(), "holding - supervisor veto: %s",
                    last_status_reason_.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "holding - %s",
                    drone_safety::ReasonName(decision.reason));
            }
            last_reason_ = decision.reason;
        }
    }

    rclcpp::Subscription<drone_interfaces::msg::ControlSetpoint>::SharedPtr setpoint_sub_;
    rclcpp::Subscription<drone_interfaces::msg::SafetyStatus>::SharedPtr status_sub_;
    rclcpp::Publisher<drone_interfaces::msg::ControlSetpoint>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    drone_interfaces::msg::ControlSetpoint last_setpoint_;
    rclcpp::Time last_setpoint_time_;
    bool has_setpoint_;

    bool last_status_veto_ = false;
    std::string last_status_reason_;
    rclcpp::Time last_status_time_;
    bool has_status_;

    drone_safety::HoldReason last_reason_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HoldFailsafeNode>());
    rclcpp::shutdown();
    return 0;
}
