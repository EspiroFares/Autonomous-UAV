#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "drone_interfaces/msg/vehicle_status.hpp"
#include "drone_interfaces/msg/control_setpoint.hpp"

using namespace std::chrono_literals;

class MockFcuNode : public rclcpp::Node {

    public: MockFcuNode(): Node("mock_fcu_node") {
        vehicle_status_pub_ = this->create_publisher<drone_interfaces::msg::VehicleStatus>("/vehicle/status", 10);

        vehicle_odom_pub_ = 
        this->create_publisher<nav_msgs::msg::Odometry>("/vehicle/odom", 10, std::bind(&MockFcuNode::setpointCallback, this, std::placeholders::_1));

        setpoint_sub_ = 
        this-> create_subscription<drone_interfaces::msg::ControlSetpoint>("/control/setpoint_validated", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&MockFcuNode::publishMockState, this));

        RCLCPP_INFO(this->get_logger(), "mock_fcu_node started");
        }
}