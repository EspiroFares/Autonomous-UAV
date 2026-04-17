#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "drone_interfaces/msg/vehicle_status.hpp"
#include "drone_interfaces/msg/control_setpoint.hpp"

using namespace std::chrono_literals;

class FcuBridgeNode : public rclcpp::Node {
    FcuBridgeNode():
    Node("fcu_bridge_node"),
    armed_(false),
    connected_(false),
    current_mode_("")
    {

        //MAVROS subs
        mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&FcuBridgeNode::MavrosStateCallback, this, std::placeholders::_1));

        mavros_odom_sub_ = this->create_subscription<macros_msgs::msg::Odometry>("/macros/local_position/odom", 10, std::bind(&FcuBridgeNode::MavrosOdomCallback, this, std::placeholders::_1));

        //ROS subs
        setpoint_sub_ = this->create_subscription<drone_interfaces::msg::ControlSetpoint>("/control/setpoint_validated", 10, std::bind(&FcuBridgeNode::SetpointCallback, this, std::placeholders::_1));


        //MAVROS pub
        cmd_vel_pub_ = this->create_publihser<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        //ROS pub
        vehicle_status_pub_ = this->create_publisher<drone_interfaces::msg::VehicleStatus>("/vehicle/status", 10);

        vehicle_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/vehicle/odom", 10);



        //setup fcu mode
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        //timer
        timer_ = this->create_wall_timer(100ms, std::bind(&FcuBridgeNode::Update, this));

        RCLCPP_INFO(this->get_logger(), "fcu_bridge_node started");

    }

    private:

        void MavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
            connected_ = msg->connected;
            armed_ = msg->armed;
            current_mode_ = msg->current_mode;
            drone_interfaces::msg::VehicleStatus status;
            status.connected = msg->connected;
            status.armed = msg->armed;
            status.offboard_ready = (msg->mode == "GUIDED" && msg->connected);
            status.mode = msg->mode;
            status.hovering = false;
            vehicle_status_pub_->publish(status);
        }

        void MacrosOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            vehicle_odom_pub_->publish(*msg);
        }

        void SetpointCallback(const drone_interfaces::msg::ControlSetpoint::SharedPtr msg) {
            last_setpoint_ = *msg:
        }

};