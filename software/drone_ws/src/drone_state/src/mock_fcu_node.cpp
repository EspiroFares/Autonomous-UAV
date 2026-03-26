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

    public:
    MockFcuNode(): Node("mock_fcu_node") {
        vehicle_status_pub_ = this->create_publisher<drone_interfaces::msg::VehicleStatus>("/vehicle/status", 10);

        vehicle_odom_pub_ = 
        this->create_publisher<nav_msgs::msg::Odometry>("/vehicle/odom", 10);

        setpoint_sub_ = 
        this-> create_subscription<drone_interfaces::msg::ControlSetpoint>("/control/setpoint_validated", 10, std::bind(&MockFcuNode::setpointCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&MockFcuNode::publishMockState, this));

        RCLCPP_INFO(this->get_logger(), "mock_fcu_node started");
        }

    private: 
    void setpointCallback(const drone_interfaces::msg::ControlSetpoint::SharedPtr msg) {
        last_setpoint_ = *msg;
    }
    void publishMockState() {
        auto now = this->get_clock()->now();

        
        drone_interfaces::msg::VehicleStatus status_msg;
        status_msg.stamp = now;
        status_msg.connected = true;
        status_msg.armed = true;
        status_msg.offboard_ready = true;
        status_msg.mode = "OFFBOARD";
        status_msg.hovering = !last_setpoint_.hold;

        vehicle_status_pub_-> publish(status_msg);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = 0.0;
        odom_msg.pose.pose.position.y = 0.0;
        odom_msg.pose.pose.position.z = 1.0;

        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
        odom_msg.pose.pose.orientation.w = 1.0;

        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.z = last_setpoint_.yaw_rate;

        vehicle_odom_pub_->publish(odom_msg);
    }

    rclcpp::Publisher<drone_interfaces::msg::VehicleStatus>::SharedPtr vehicle_status_pub_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_odom_pub_;
    rclcpp::Subscription<drone_interfaces::msg::ControlSetpoint>::SharedPtr setpoint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    drone_interfaces::msg::ControlSetpoint last_setpoint_;
    };

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockFcuNode>());
  rclcpp::shutdown();
  return 0;
};