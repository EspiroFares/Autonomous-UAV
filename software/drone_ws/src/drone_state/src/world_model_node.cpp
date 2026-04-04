#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "drone_interfaces/msg/target_state.hpp"
#include "drone_interfaces/msg/vehicle_status.hpp"

using namespace std::chrono_literals;

class WorldModelNode : public rclcpp::Node {

public:
  WorldModelNode()
  : Node("world_model_node"),
    confidence_threshold_(0.5f),
    has_target_(false),
    has_vehicle_status_(false),
    has_vehicle_odom_(false)
  {
    vehicle_status_sub_ = this->create_subscription<drone_interfaces::msg::VehicleStatus>(
      "/vehicle/status", 10,
      std::bind(&WorldModelNode::VehicleStatusCallback, this, std::placeholders::_1));

    vehicle_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vehicle/odom", 10,
      std::bind(&WorldModelNode::VehicleOdomCallback, this, std::placeholders::_1));

    target_state_sub_ = this->create_subscription<drone_interfaces::msg::TargetState>(
      "/target/state", 10,
      std::bind(&WorldModelNode::TargetStateCallback, this, std::placeholders::_1));

    target_valid_pub_ = this->create_publisher<std_msgs::msg::Bool>("/world/target_valid", 10);

    target_pos_relative_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
      "/world/target_pos_relative", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&WorldModelNode::PublishWorldState, this));

    RCLCPP_INFO(this->get_logger(), "world_model_node started");
  }

private:
  void VehicleStatusCallback(const drone_interfaces::msg::VehicleStatus::SharedPtr msg) {
    last_vehicle_status_ = *msg;
    has_vehicle_status_ = true;
  }

  void VehicleOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_vehicle_odom_ = *msg;
    has_vehicle_odom_ = true;
  }

  void TargetStateCallback(const drone_interfaces::msg::TargetState::SharedPtr msg) {
    last_target_state_ = *msg;
    has_target_ = true;
  }

  void PublishWorldState() {
    if (!has_vehicle_status_ || !has_vehicle_odom_ || !has_target_) {
      return;
    }

    bool vehicle_ready = last_vehicle_status_.connected &&
                         last_vehicle_status_.armed &&
                         last_vehicle_status_.offboard_ready;

    bool target_valid = vehicle_ready &&
                        last_target_state_.detected &&
                        last_target_state_.confidence >= confidence_threshold_;

    // Publish target validity
    std_msgs::msg::Bool valid_msg;
    valid_msg.data = target_valid;
    target_valid_pub_->publish(valid_msg);

    // Publish relative position (forward = x, lateral = y, vertical = z)
    // yaw_error is the horizontal angular offset to the target
    // distance_estimate is the estimated range
    geometry_msgs::msg::Point pos_msg;
    if (target_valid) {
      pos_msg.x = last_target_state_.distance_estimate * std::cos(last_target_state_.yaw_error);
      pos_msg.y = last_target_state_.distance_estimate * std::sin(last_target_state_.yaw_error);
      pos_msg.z = 0.0;
    } else {
      pos_msg.x = 0.0;
      pos_msg.y = 0.0;
      pos_msg.z = 0.0;
    }
    target_pos_relative_pub_->publish(pos_msg);
  }

  // Subscribers
  rclcpp::Subscription<drone_interfaces::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_odom_sub_;
  rclcpp::Subscription<drone_interfaces::msg::TargetState>::SharedPtr target_state_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_valid_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pos_relative_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Latest received messages
  drone_interfaces::msg::VehicleStatus last_vehicle_status_;
  nav_msgs::msg::Odometry last_vehicle_odom_;
  drone_interfaces::msg::TargetState last_target_state_;

  // Readiness flags
  bool has_vehicle_status_;
  bool has_vehicle_odom_;
  bool has_target_;

  float confidence_threshold_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldModelNode>());
  rclcpp::shutdown();
  return 0;
}
