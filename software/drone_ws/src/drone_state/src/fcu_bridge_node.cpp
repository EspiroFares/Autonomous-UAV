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
    public:
    FcuBridgeNode():
    Node("fcu_bridge_node"),
    armed_(false),
    connected_(false),
    current_mode_("")
    {

        //MAVROS subs
        mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&FcuBridgeNode::MavrosStateCallback, this, std::placeholders::_1));

        mavros_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom", 10, std::bind(&FcuBridgeNode::MavrosOdomCallback, this, std::placeholders::_1));

        //ROS subs
        setpoint_sub_ = this->create_subscription<drone_interfaces::msg::ControlSetpoint>("/control/setpoint_validated", 10, std::bind(&FcuBridgeNode::SetpointCallback, this, std::placeholders::_1));


        //MAVROS pub
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

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
            current_mode_ = msg->mode  ;
            drone_interfaces::msg::VehicleStatus status;
            status.connected = msg->connected;
            status.armed = msg->armed;
            status.offboard_ready = (msg->mode == "GUIDED" && msg->connected);
            status.mode = msg->mode;
            status.hovering = false;
            vehicle_status_pub_->publish(status);
        }

        void MavrosOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            vehicle_odom_pub_->publish(*msg);
        }

        void SetpointCallback(const drone_interfaces::msg::ControlSetpoint::SharedPtr msg) {
            last_setpoint_ = *msg;
        }

        void Update() {
            //Initialte GUIDED mode
            if (connected_ && current_mode_ != "GUIDED") {
                auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                request->custom_mode = "GUIDED";
                set_mode_client_->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "Reqeuesting GUIDED mode.........");
            }

            geometry_msgs::msg::Twist cmd;

            if (last_setpoint_.hold || !armed_) {
                cmd.linear.x  = 0.0;
                cmd.linear.y  = 0.0;
                cmd.linear.z  = 0.0;
                cmd.angular.z  = 0.0;
            }else {
                cmd.linear.x  = last_setpoint_.vx;
                cmd.linear.y  = last_setpoint_.vy;
                cmd.linear.z  = last_setpoint_.vz;
                cmd.angular.z  = last_setpoint_.yaw_rate;
            }

            cmd_vel_pub_->publish(cmd);
        }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_sub_;
    rclcpp::Subscription<drone_interfaces::msg::ControlSetpoint>::SharedPtr setpoint_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<drone_interfaces::msg::VehicleStatus>::SharedPtr vehicle_status_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_odom_pub_;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool armed_;
    bool connected_;
    std::string current_mode_;
    drone_interfaces::msg::ControlSetpoint last_setpoint_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FcuBridgeNode>());
    rclcpp::shutdown();
    return 0;
}