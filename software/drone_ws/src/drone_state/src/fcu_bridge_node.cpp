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

        //mavros subs
        mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&FcuBridgeNode::MavrosStateCallback, this, std::placeholders::_1));

        mavros_odom_sub_ = this->create_subscription<macros_msgs::msg::Odometry>("/macros/local_position/odom", 10, std::bind(&FcuBridgeNode::MacrosOdomCallback, this, std::placeholders::_1));

        //ros subs
        setpoint_sub_ = this->create_subscription<drone_interfaces::msg::ControlSetpoint>("/control/setpoint_validated", 10, std::bind(&FcuBridgeNode::SetpointCallback, this, std::placeholders::_1));




    }
};