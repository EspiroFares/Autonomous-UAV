// Final safety gate on the control setpoint: rejects NaN/Inf (forcing a hold)
// and clamps every axis to a hard velocity envelope before it reaches the FC.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "drone_interfaces/msg/control_setpoint.hpp"
#include "drone_control/setpoint_limits.hpp"

class SetpointValidationNode : public rclcpp::Node {
public:
    SetpointValidationNode() : Node("setpoint_validation_node") {

        sub_ = this->create_subscription<drone_interfaces::msg::ControlSetpoint>(
            "/control/setpoint_raw", 10,
            std::bind(&SetpointValidationNode::OnSetpoint, this, std::placeholders::_1));

        pub_ = this->create_publisher<drone_interfaces::msg::ControlSetpoint>(
            "/control/setpoint_validated", 10);

        RCLCPP_INFO(this->get_logger(), "setpoint_validation_node started");
    }

private:
    void OnSetpoint(const drone_interfaces::msg::ControlSetpoint::SharedPtr msg) {
        // The envelope itself lives in setpoint_limits.hpp so it can be unit
        // tested without spinning up ROS. This node is only the plumbing.
        pub_->publish(drone_control::ValidateSetpoint(*msg, limits_));
    }

    drone_control::VelocityLimits limits_;

    rclcpp::Subscription<drone_interfaces::msg::ControlSetpoint>::SharedPtr sub_;
    rclcpp::Publisher<drone_interfaces::msg::ControlSetpoint>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetpointValidationNode>());
    rclcpp::shutdown();
    return 0;
}
