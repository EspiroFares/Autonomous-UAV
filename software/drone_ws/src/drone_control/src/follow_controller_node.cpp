#include <algorithm>
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
    has_odom_(false),
    prev_dist_(0.0),
    person_vel_filt_(0.0),
    last_vx_cmd_(0.0),
    vx_idx_(0),
    has_prev_dist_(false)
    {

            target_valid_sub_ = this->create_subscription<std_msgs::msg::Bool>("/mission/follow_enabled", 10, std::bind(&FollowControllerNode::FollowEnabledCallback, this, std::placeholders::_1));

            target_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/world/target_pos_relative", 10, std::bind(&FollowControllerNode::TargetPosCallback, this, std::placeholders::_1));

            height_sub_ = this->create_subscription<std_msgs::msg::Float32>("/vehicle/height", 10,std::bind(&FollowControllerNode::HeightCallback, this,std::placeholders::_1));

            setpoint_pub_ = this->create_publisher<drone_interfaces::msg::ControlSetpoint>("/control/setpoint_raw", 10);

            timer_ = this->create_wall_timer(50ms, std::bind(&FollowControllerNode::Update, this));

             RCLCPP_INFO(this->get_logger(), "follow_controller_node started");

            last_pos_time_ = this->now();
            last_height_time_ = this->now();
            last_follow_time_ = this->now();

            prev_dist_time_ = this->now();
            for (int i = 0; i < 7; i++) vx_hist_[i] = 0.0;

            this->declare_parameter("kp_yaw", 0.4);
            this->declare_parameter("max_yaw_rate", 0.3);
            this->declare_parameter("yaw_deadband", 0.09);

            this->declare_parameter("desired_distance", 2.0);
            this->declare_parameter("kp_vx", 0.5);
            this->declare_parameter("kff", 0.5);
            this->declare_parameter("max_vx", 1.0);
            this->declare_parameter("vx_deadband", 0.35);
            this->declare_parameter("vx_slew_fast", 0.30);
            this->declare_parameter("vx_slew_slow", 0.06);
            this->declare_parameter("slew_switch_dist", 0.4);

            this->declare_parameter("desired_altitude", 1.5);
            this->declare_parameter("kp_z", 0.4);
            this->declare_parameter("max_vz", 0.6);

            this->declare_parameter("target_fresh_timeout", 0.5);

            this->declare_parameter("min_distance", 1.0);
            this->declare_parameter("k_approach", 1.2);
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
            
            //Make all the params ROS params
            const double KP_YAW        = this->get_parameter("kp_yaw").as_double();
            const double MAX_YAW_RATE  = this->get_parameter("max_yaw_rate").as_double();
            const double YAW_DEADBAND  = this->get_parameter("yaw_deadband").as_double();
            const double DESIRED_DISTANCE = this->get_parameter("desired_distance").as_double();
            const double KP_VX         = this->get_parameter("kp_vx").as_double();
            const double KFF           = this->get_parameter("kff").as_double();
            const double MAX_VX        = this->get_parameter("max_vx").as_double();
            const double VX_DEADBAND   = this->get_parameter("vx_deadband").as_double();
            const double VX_SLEW_FAST  = this->get_parameter("vx_slew_fast").as_double();
            const double VX_SLEW_SLOW  = this->get_parameter("vx_slew_slow").as_double();
            const double SLEW_SWITCH_DIST = this->get_parameter("slew_switch_dist").as_double();
            const double DESIRED_ALTITUDE = this->get_parameter("desired_altitude").as_double();
            const double KP_Z          = this->get_parameter("kp_z").as_double();
            const double MAX_VZ        = this->get_parameter("max_vz").as_double();

            const double MIN_DISTANCE = this->get_parameter("min_distance").as_double();
            const double K_APPROACH   = this->get_parameter("k_approach").as_double();

            drone_interfaces::msg::ControlSetpoint setpoint;

            const auto now = this->now();

            const double TARGET_FRESH_TIMEOUT =
                this->get_parameter("target_fresh_timeout").as_double();

            const bool pos_fresh =
                (now - last_pos_time_).seconds() < TARGET_FRESH_TIMEOUT;

            const bool height_fresh =
                has_odom_ &&
                std::isfinite(current_z_) &&
                (now - last_height_time_).seconds() < 0.5;

            const bool follow_fresh =
                (now - last_follow_time_).seconds() < 0.5;

            // Död eller ogiltig lidar: stoppa alla kommandon.
            if (!height_fresh) {
                setpoint.vx = 0.0;
                setpoint.vy = 0.0;
                setpoint.vz = 0.0;
                setpoint.yaw_rate = 0.0;
                setpoint.hold = true;

                person_vel_filt_ = 0.0;
                last_vx_cmd_ = 0.0;
                has_prev_dist_ = false;

                setpoint_pub_->publish(setpoint);
                return;
            }

            // Yttre lidarbaserad AGL-reglering.
            // Körs även under IDLE och TARGET_LOST.
            const double vz_command = std::clamp(
                KP_Z * (DESIRED_ALTITUDE - current_z_),
                -MAX_VZ,
                MAX_VZ
            );

            const bool follow_ok =
                follow_enabled_ &&
                follow_fresh &&
                pos_fresh;

            if (!follow_ok) {
                setpoint.vx = 0.0;
                setpoint.vy = 0.0;
                setpoint.vz = vz_command;
                setpoint.yaw_rate = 0.0;
                setpoint.hold = false;

                person_vel_filt_ *= 0.85;
                last_vx_cmd_ = 0.0;
                vx_hist_[vx_idx_] = 0.0;
                vx_idx_ = (vx_idx_ + 1) % 7;

                setpoint_pub_->publish(setpoint);
                return;
            }

            double yaw_error = std::atan2(target_y_, target_x_);
            double yaw_rate = 0.0;
            if (std::abs(yaw_error) > YAW_DEADBAND) {
                yaw_rate = std::clamp(KP_YAW * -yaw_error, -MAX_YAW_RATE, MAX_YAW_RATE);
            }

            double dist = std::sqrt(target_x_ * target_x_ + target_y_ * target_y_);

            // Personens hastighet = relativ avståndsändring + vårt kommando
            // från ~0.7 s sedan (perception-latens + FC-svar, tidsmatchat)
            double dt = (this->now() - prev_dist_time_).seconds();
            if (!has_prev_dist_ || dt > 0.6) {
                // första mätningen eller för långt gap: nollställ baslinjen
                prev_dist_ = dist;
                prev_dist_time_ = this->now();
                has_prev_dist_ = true;
            } else if (std::abs(dist - prev_dist_) > 0.001 && dt > 0.05) {
                double raw_rate = (dist - prev_dist_) / dt;
                double vx_delayed = vx_hist_[vx_idx_];
                double person_vel_raw = std::clamp(raw_rate + vx_delayed, -2.0, 2.0);
                person_vel_filt_ = 0.6 * person_vel_filt_ + 0.4 * person_vel_raw;
                prev_dist_ = dist;
                prev_dist_time_ = this->now();
            }

            double dist_error = dist - DESIRED_DISTANCE;
            double vx = KP_VX * dist_error + KFF * person_vel_filt_;

            // Hold-zon: inom bandet och personen still -> ligg kvar
            if (std::abs(dist_error) < VX_DEADBAND && std::abs(person_vel_filt_) < 0.15) {
                vx = 0.0;
            }

            // Begränsa controllerns vanliga framåtkommando.
            vx = std::clamp(vx, -MAX_VX, MAX_VX);

            // Adaptivt framåt-tak:
            // 0 m/s vid MIN_DISTANCE och linjärt ökande längre bort.
            // Negativ vx (reträtt) påverkas inte.
            const double max_approach = std::clamp(
                K_APPROACH * (dist - MIN_DISTANCE),
                0.0,
                MAX_VX
            );

            vx = std::min(vx, max_approach);

            // Asymmetrisk slew:
            // snabb broms/reträtt, långsammare acceleration framåt.
            const double slew_fwd =
                (std::abs(dist_error) > SLEW_SWITCH_DIST)
                    ? VX_SLEW_FAST
                    : VX_SLEW_SLOW;

            const double slew_brake = VX_SLEW_FAST * 2.0;

            vx = std::clamp(
                vx,
                last_vx_cmd_ - slew_brake,
                last_vx_cmd_ + slew_fwd
            );

            // Hårt säkerhetstak EFTER slew.
            // Slew får aldrig återinföra framåthastighet över taket.
            vx = std::min(vx, max_approach);

            last_vx_cmd_ = vx;

            vx_hist_[vx_idx_] = vx;
            vx_idx_ = (vx_idx_ + 1) % 7;

        
            setpoint.vx = vx;
            setpoint.vy = 0.0;
            setpoint.vz = vz_command;
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
        rclcpp::Time prev_dist_time_;

        double prev_dist_;
        double person_vel_filt_;
        double last_vx_cmd_;
        double vx_hist_[7];
        int vx_idx_;
        bool has_prev_dist_;

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

