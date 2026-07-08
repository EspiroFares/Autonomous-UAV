#include <chrono>
#include<functional>                                                                      
#include<memory>                                                                          
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "drone_interfaces/msg/control_setpoint.hpp"  

#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class FollowControllerNode : public rclcpp::Node {
    public:
    FollowControllerNode():
    Node("follow_controller_node"),
    follow_enabled_(false),
    target_x_(0.0),
    target_y_(0.0),
    current_z_(0.0),
    has_odom_(false)
    {

            target_valid_sub_ = this->create_subscription<std_msgs::msg::Bool>("/mission/follow_enabled", 10, std::bind(&FollowControllerNode::FollowEnabledCallback, this, std::placeholders::_1));

            target_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/world/target_pos_relative", 10, std::bind(&FollowControllerNode::TargetPosCallback, this, std::placeholders::_1));

            height_sub_ = this->create_subscription<std_msgs::msg::Float32>("/vehicle/height", 10,std::bind(&FollowControllerNode::HeightCallback, this,std::placeholders::_1));

            setpoint_pub_ = this->create_publisher<drone_interfaces::msg::ControlSetpoint>("/control/setpoint_raw", 10);

            timer_ = this->create_wall_timer(100ms, std::bind(&FollowControllerNode::Update, this));

             RCLCPP_INFO(this->get_logger(), "follow_controller_node started");

            last_pos_time_ = this->now();
            last_height_time_ = this->now();
            last_follow_time_ = this->now();
    }

    private:
        void FollowEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg){
            follow_enabled_ = msg->data;
            last_follow_time_ = this->now();
        }

        void TargetPosCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
            target_x_ = msg->x;
            target_y_ = msg->y;
            last_pos_time_ = this->now();
        }
        //(void)msg just for now
       void HeightCallback(const std_msgs::msg::Float32::SharedPtr msg){
            current_z_ = msg->data;
            has_odom_ = true;
            last_height_time_ = this->now();
        }

        void Update() {
            drone_interfaces::msg::ControlSetpoint setpoint;

            bool pos_fresh    = (this->now() - last_pos_time_).seconds() < 0.5;
            bool height_fresh = has_odom_ && (this->now() - last_height_time_).seconds() < 0.5;
            bool follow_fresh = (this->now() - last_follow_time_).seconds() < 0.5;

            bool follow_ok = follow_enabled_ && follow_fresh && pos_fresh;

            if (!follow_ok) {
                setpoint.vx = 0.0;
                setpoint.vy = 0.0;
                setpoint.yaw_rate = 0.0;

                double vz = 0.0;
                if (height_fresh) {
                    double z_error = 1.5 - current_z_;
                    vz = std::clamp(0.8 * z_error, -0.2, 0.2);
                    setpoint.hold = false;
                } else {
                    setpoint.hold = true;
                }
                setpoint.vz = vz;
                setpoint_pub_->publish(setpoint);
                return;
            }

            const double KP_YAW = 0.4;
            const double KP_VX = 0.3;
            const double MAX_YAW_RATE = 0.3;
            const double MAX_VX = 0.3;
            const double DESIRED_DISTANCE = 1.5;
            const double YAW_DEADBAND = 0.09;   // ~5 grader — reagera inte på småfel
            const double VX_DEADBAND = 0.15;    // 15 cm

            const double KP_Z = 0.8;
            const double MAX_VZ = 0.2;
            const double DESIRED_ALTITUDE = 1.5;


            double yaw_error = std::atan2(target_y_, target_x_);
            double yaw_rate = 0.0;
            if (std::abs(yaw_error) > YAW_DEADBAND) {
                yaw_rate = std::clamp(KP_YAW * -yaw_error, -MAX_YAW_RATE, MAX_YAW_RATE);
            }

            double dist = std::sqrt(target_x_ * target_x_ + target_y_ * target_y_);
            double vx = 0.0;
            double vz = 0.0;
            if (height_fresh) {
                double z_error = DESIRED_ALTITUDE - current_z_;
                vz = std::clamp(KP_Z * z_error, -MAX_VZ, MAX_VZ);
            }
            setpoint.vz = vz;
            if (std::abs(dist - DESIRED_DISTANCE) > VX_DEADBAND) {
                vx = std::clamp(KP_VX * (dist - DESIRED_DISTANCE), -MAX_VX, MAX_VX);
            }

            setpoint.vx = vx;
            setpoint.vy = 0.0;
            setpoint.yaw_rate = yaw_rate;
            setpoint.hold = false;
            setpoint_pub_->publish(setpoint);
            
        }

        

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_valid_sub_;

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_pos_sub_;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_;

        rclcpp::Publisher<drone_interfaces::msg::ControlSetpoint>::SharedPtr setpoint_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Time last_pos_time_;
        rclcpp::Time last_height_time_;
        rclcpp::Time last_follow_time_;

        bool follow_enabled_;
        double target_x_;
        double target_y_;
        double current_z_;
        bool has_odom_;
        

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowControllerNode>()); 
     rclcpp::shutdown();
     return 0;  
}

