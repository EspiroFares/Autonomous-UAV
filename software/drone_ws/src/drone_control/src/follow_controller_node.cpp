#include <chrono>
#include<functional>                                                                      
#include<memory>                                                                          
#include <cmath>
                                                                                            
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"                                                     
#include "nav_msgs/msg/odometry.hpp"
#include "drone_interfaces/msg/control_setpoint.hpp"  

using namespace std::chrono_literals;

class FollowControllerNode : public rclcpp::Node {
    public:
    FollowControllerNode():
    Node("follow_controller_node"),
    follow_enabled_(false),
    target_x_(0.0),
    target_y_(0.0)
    {

            target_valid_sub_ = this->create_subscription<std_msgs::msg::Bool>("/mission/follow_enabled", 10, std::bind(&FollowControllerNode::FollowEnabledCallback, this, std::placeholders::_1));

            target_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/world/target_pos_relative", 10, std::bind(&FollowControllerNode::TargetPosCallback, this, std::placeholders::_1));

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/vehicle/odom", 10, std::bind(&FollowControllerNode::OdomCallback, this, std::placeholders::_1));

            setpoint_pub_ = this->create_publisher<drone_interfaces::msg::ControlSetpoint>("/control/setpoint_raw", 10);

            timer_ = this->create_wall_timer(100ms, std::bind(&FollowControllerNode::Update, this));

             RCLCPP_INFO(this->get_logger(), "follow_controller_node started");
    }

    private:
        void FollowEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg){
            follow_enabled_ = msg->data;
        }

        void TargetPosCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
            target_x_ = msg->x;
            target_y_ = msg->y;
        }
        //(void)msg just for now
        void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
            (void)msg; 
        }

        void Update() {
            drone_interfaces::msg::ControlSetpoint setpoint;

            if (!follow_enabled_) {
                setpoint.vx = 0.0;
                setpoint.vy = 0.0;
                setpoint.vz = 0.0;
                setpoint.yaw_rate = 0.0;
                setpoint.hold = true;
                setpoint_pub_->publish(setpoint);
                return;
            }

            //adjust these for later
            const double KP_YAW = 1.2;
            const double KP_VX = 0.5;
            const double MAX_YAW_RATE = 1.0;
            const double MAX_VX = 0.5;

            //clamp yaw rate in bound
            double yaw_rate = std::clamp(KP_YAW * target_y_, -MAX_YAW_RATE , MAX_YAW_RATE);
            double vx = std::clamp(KP_VX * target_x_ , -MAX_VX, MAX_VX);

            setpoint.vx = vx;
            setpoint.vy = 0.0;
            setpoint.vz = 0.0;
            setpoint.yaw_rate = yaw_rate;
            setpoint.hold = false;
            setpoint_pub_->publish(setpoint);
        }

        

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_valid_sub_;

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_pos_sub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        rclcpp::Publisher<drone_interfaces::msg::ControlSetpoint>::SharedPtr setpoint_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        bool follow_enabled_;
        double target_x_;
        double target_y_;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowControllerNode>()); 
     rclcpp::shutdown();
     return 0;  
}

