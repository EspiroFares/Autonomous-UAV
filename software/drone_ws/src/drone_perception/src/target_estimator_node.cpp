#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "drone_interfaces/msg/track.hpp"
#include "drone_interfaces/msg/target_state.hpp"

class TargetEstimatorNode : public rclcpp::Node {
public:
    TargetEstimatorNode() : Node("target_estimator_node") {
        sub_ = this->create_subscription<drone_interfaces::msg::Track>("/target/track", 10, std::bind(&TargetEstimatorNode::on_track, this, std::placeholders::_1));

        pub_ = this->create_publisher<drone_interfaces::msg::TargetState>("/target/state", 10);

        RCLCPP_INFO(this->get_logger(), "target_estimator_node started"); 
    }

private:
    void on_track(const drone_interfaces::msg::Track::SharedPtr msg) {
        drone_interfaces::msg::TargetState target;

        if (!msg->valid) {
            target.detected = false;
            target.confidence = 0.0f;
            target.yaw_error = 0.0f;
            target.distance_estimate = 0.0f;
            pub_->publish(target);
            return;
        }

        const float known_shoulder_width = 0.45f;
        const float focal_length = 600.0f;
        const float image_width = 640.0f; 

        float shoulder_width_px = msg->width * image_width;
        float distance = (known_shoulder_width * focal_length) / (shoulder_width_px + 1e-6f); 
        float yaw_error = (msg->center_x - 0.5f) * 2.0f;

        target.detected = true;
        target.confidence = 0.9f;
        target.yaw_error = yaw_error;
        target.distance_estimate = distance;

        pub_->publish(target);
    }

    rclcpp::Subscription<drone_interfaces::msg::Track>::SharedPtr sub_;
    rclcpp::Publisher<drone_interfaces::msg::TargetState>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}