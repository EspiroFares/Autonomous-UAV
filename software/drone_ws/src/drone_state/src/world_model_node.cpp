#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "drone_interfaces/msg/target_state.hpp"
#include "drone_interfaces/msg/vehicle_status.hpp"

using namespace std::chrono_literals;

class WorldModelNode : public rclcpp::Node {

public: WorldModelNode():   
    Node(world_model_node),
    confidence_threshold_(0.5f),
    has_target_(false),
    has_vehicle_status_(false),
    has_vehicle_odom_(false)
  {
    vehicle_status_sub_ = this-> create_subscription<drone_interfaces::msg::VehicleState>("/vehicle/status", 10, std::bind(&WorldModelNode::VehicleStatusCallback, this, std::placeholders::_1));

    vehicle_odom_sub_ = this->create_subsciption<nav_msgs::msg::Odometry>("/vehicle/odom", 10, 
    std::bind(&WorldModelNode::VehicleOdomCallback, this, std::placeholders::_1));

    target_state_sub_ = this->create_subscription<drone_interfaces::msg::TargetState>("/target/state", 10, 
    std::bind(&WorldModelNode::TargetStateCallback, this, std::placeholders::_1));


    target_valid_pub_ -> this.create_publisher<std_msgs::msg::Bool>("/world_target_node",10);

    target_pos_relativ_pub_ -> this.create_publisher<geometry_msgs::msg::Point>()("/world/target_pos_relativ", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&WorldModelNode::PublishWorldState, this));

    RCLCPP_INFO(this->get_logger(), "world_model_node started");

  }
  
  private: 
    void VehicleStatusCallback(const drone_interfaces::msg::VehicleStatus: SharedPtr msg) {
      last_vehicle_status_ = *msg;
      has_vehicle_status_ = true;
    }
    void VehicleOdomCallback(const nav_msgs::msg::Odemetry: SharedPtr msg){
      last_vehicle_odom = *msg;
      has_vehicle_odom = true;
    }
    void TargetStateCallback(const drone_interfaces::msg::TargetState: SharedPtr msg) {
      last_state = *msg;
      has_state = true;
    }
    
    void PublishWorldState(){
      

    }
  
  };
