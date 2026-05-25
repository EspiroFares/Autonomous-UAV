#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "drone_interfaces/msg/control_setpoint.hpp"

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
        drone_interfaces::msg::ControlSetpoint out;

        // NaN/Inf check — force hold if any field is invalid
        if (!std::isfinite(msg->vx) || !std::isfinite(msg->vy) ||
            !std::isfinite(msg->vz) || !std::isfinite(msg->yaw_rate)) {
            out.vx = 0.0; out.vy = 0.0; out.vz = 0.0; out.yaw_rate = 0.0;
            out.hold = true;
            pub_->publish(out);
            return;
        }

        out.vx       = std::clamp(msg->vx,       -1.0f, 1.0f);
        out.vy       = std::clamp(msg->vy,       -1.0f, 1.0f);
        out.vz       = std::clamp(msg->vz,       -0.5f, 0.5f);
        out.yaw_rate = std::clamp(msg->yaw_rate, -1.5f, 1.5f);
        out.hold     = msg->hold;

        pub_->publish(out);
    }

    rclcpp::Subscription<drone_interfaces::msg::ControlSetpoint>::SharedPtr sub_;
    rclcpp::Publisher<drone_interfaces::msg::ControlSetpoint>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetpointValidationNode>());
    rclcpp::shutdown();
    return 0;
}
