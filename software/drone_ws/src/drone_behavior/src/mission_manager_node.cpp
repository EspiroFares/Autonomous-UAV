  #include <chrono>
  #include <functional>                                                         
  #include <memory>                                                           

  #include "rclcpp/rclcpp.hpp"
  #include "std_msgs/msg/bool.hpp"
  #include "std_msgs/msg/string.hpp"                                            
  #include "geometry_msgs/msg/point.hpp"
  #include "drone_interfaces/msg/mission_state.hpp"                             
                                                                                
  using namespace std::chrono_literals;
                                                                                
  enum class MissionState {                                                   
    IDLE,
    SEARCH,
    TRACKING,                                                                   
    FOLLOWING,
    TARGET_LOST,                                                                
    SAFETY_HOLD                                                               
  };

  class MissionManagerNode : public rclcpp:Node {
    public:
    MissionManagerNode(): 
    Node("mission_manager_node"), 
    state_(MissionState:IDLE),
    target_valid_(false),
    target_pos_x_(0,0),
    target_pos_y_(0,0)

    {
        target_valid_sub_ = this->create_subscription<std_msgs::msg::Bool>("/world/target_valid", 10, std::bind(&MissionManagerNode::TargetValidCallback, this, srd::placerholders::_1));

        target_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/world/target_pos_relative", 10, std::bind(&MissionManagerNode::TargetPosCallback, this, srd::placerholders::_1));

        mission_state_pub_ = this->create_publisher<drone_interfaces::msg::MissionState>("/mission/state", 10);

        follow_enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>("/mission/follow_enabled", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&MissionManagerNode::Update, this));

        RCLCPP_INFO(this->get_logger(), "mission_manager_node started");

    }
  }
       