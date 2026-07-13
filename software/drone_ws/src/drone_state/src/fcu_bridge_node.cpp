#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "mavros_msgs/msg/state.hpp"

#include "mavros_msgs/srv/set_mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "drone_interfaces/msg/vehicle_status.hpp"
#include "drone_interfaces/msg/control_setpoint.hpp"


using namespace std::chrono_literals;

class FcuBridgeNode : public rclcpp::Node {
    public:
    FcuBridgeNode():
    Node("fcu_bridge_node"),
    armed_(false),
    connected_(false),
    current_mode_(""),
    has_setpoint_(false)
    {

        //MAVROS subs — must match MAVROS publisher QoS (BEST_EFFORT)
        auto mavros_qos = rclcpp::QoS(10).best_effort();

        mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", mavros_qos, std::bind(&FcuBridgeNode::MavrosStateCallback, this, std::placeholders::_1));

        mavros_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom", mavros_qos, std::bind(&FcuBridgeNode::MavrosOdomCallback, this, std::placeholders::_1));

        rangefinder_sub_ = this->create_subscription<sensor_msgs::msg::Range>("/mavros/rangefinder/rangefinder", mavros_qos,std::bind(&FcuBridgeNode::RangefinderCallback, this, std::placeholders::_1));

        //ROS subs
        setpoint_sub_ = this->create_subscription<drone_interfaces::msg::ControlSetpoint>("/control/setpoint_validated", 10, std::bind(&FcuBridgeNode::SetpointCallback, this, std::placeholders::_1));


        //MAVROS pub
        cmd_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);

        //ROS pub
        vehicle_status_pub_ = this->create_publisher<drone_interfaces::msg::VehicleStatus>("/vehicle/status", 10);

        vehicle_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/vehicle/odom", 10);

        vehicle_height_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vehicle/height", 10);



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
            status.offboard_ready = msg->guided && msg->connected; 
            status.mode = msg->mode;
            status.hovering = false;
            vehicle_status_pub_->publish(status);
        }

        void MavrosOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            vehicle_odom_pub_->publish(*msg);
        }

        void RangefinderCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
            std_msgs::msg::Float32 height;
            height.data = msg->range;
            vehicle_height_pub_->publish(height);
        }

        void SetpointCallback(const drone_interfaces::msg::ControlSetpoint::SharedPtr msg) {
            last_setpoint_ = *msg;
            last_setpoint_time_ = this->now();
            has_setpoint_ = true;
        }

        void Update() {

            bool setpoint_fresh = has_setpoint_ && (this->now() - last_setpoint_time_).seconds() < 0.5;
            //Initialte GUIDED mode
            mavros_msgs::msg::PositionTarget cmd;
            cmd.header.stamp = this->now();
            cmd.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
            // Använd vx, vy, yaw_rate. Ignorera ALLT om Z -> FC håller höjden själv.
            cmd.type_mask =
                mavros_msgs::msg::PositionTarget::IGNORE_PX |
                mavros_msgs::msg::PositionTarget::IGNORE_PY |
                mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                mavros_msgs::msg::PositionTarget::IGNORE_YAW;

            if (last_setpoint_.hold || !armed_ || !setpoint_fresh) {
                cmd.velocity.x = 0.0; cmd.velocity.y = 0.0; cmd.yaw_rate = 0.0;
            } else {
                cmd.velocity.x = last_setpoint_.vx;
                cmd.velocity.y = last_setpoint_.vy;
                cmd.yaw_rate   = last_setpoint_.yaw_rate;
            }
            cmd_pub_->publish(cmd);
        }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_sub_;
    rclcpp::Subscription<drone_interfaces::msg::ControlSetpoint>::SharedPtr setpoint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangefinder_sub_;


    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr cmd_pub_;
    rclcpp::Publisher<drone_interfaces::msg::VehicleStatus>::SharedPtr vehicle_status_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vehicle_height_pub_;

    

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_setpoint_time_;
    

    bool has_setpoint_;
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