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

  class MissionManagerNode : public rclcpp::Node {
    public:
    MissionManagerNode(): 
    Node("mission_manager_node"), 
    state_(MissionState::IDLE),
    target_valid_(false),
    target_pos_x_(0.0),
    target_pos_y_(0.0)

    {
        target_valid_sub_ = this->create_subscription<std_msgs::msg::Bool>("/world/target_valid", 10, std::bind(&MissionManagerNode::TargetValidCallback, this, std::placeholders::_1));

        target_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/world/target_pos_relative", 10, std::bind(&MissionManagerNode::TargetPosCallback, this, std::placeholders::_1));

        mission_state_pub_ = this->create_publisher<drone_interfaces::msg::MissionState>("/mission/state", 10);

        follow_enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>("/mission/follow_enabled", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&MissionManagerNode::Update, this));

        RCLCPP_INFO(this->get_logger(), "mission_manager_node started");

    }

    private:
        void TargetValidCallback(const std_msgs::msg::Bool::SharedPtr msg){
            target_valid_ = msg->data;
        }
        void TargetPosCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
            target_pos_x_ = msg->x;
            target_pos_y_ = msg->y;
        }
        void Update() {
            MissionState next_state = state_;

            switch(state_) {
                case MissionState::IDLE:
                    if (target_valid_) {
                        next_state = MissionState::TRACKING;
                    }
                    break;
                case MissionState::SEARCH:
                    if (target_valid_) {
                        next_state = MissionState::TRACKING;
                    }
                    break;
                case MissionState::TRACKING:
                    if (!target_valid_) {
                        next_state = MissionState::TARGET_LOST;
                    }
                    else {
                        next_state = MissionState::FOLLOWING;
                    }
                    break;

                case MissionState::FOLLOWING:
                    if (!target_valid_) {
                        next_state = MissionState::TARGET_LOST;
                    }
                    break;
                
                case MissionState::TARGET_LOST:
                    if (target_valid_) {
                        next_state = MissionState::TRACKING;
                    }
                    break;
                case MissionState::SAFETY_HOLD:
                    break;

                }

            if (next_state != state_) {
                RCLCPP_INFO(this->get_logger(), "State: %s -> %s", StateToString(state_).c_str(),StateToString(next_state).c_str());
                state_ = next_state;
            }

        PublishState();   
      
        }

        std::string StateToString(MissionState state) {
            switch (state) {
                case MissionState::IDLE: return "IDLE";
                case MissionState::SEARCH: return "SEARCH";
                case MissionState::TRACKING: return "TRACKING";
                case MissionState::FOLLOWING: return "FOLLOWING";
                case MissionState::TARGET_LOST: return "TARGET_LOST";
                case MissionState::SAFETY_HOLD: return "SAFETY_HOLD";
                default: return "UNKNOWN";
            }
        }
        void PublishState() {
            drone_interfaces::msg::MissionState state_msg;
            state_msg.state = StateToString(state_);
            state_msg.follow_enabled = (state_ == MissionState::FOLLOWING);
            state_msg.target_valid = target_valid_;
            mission_state_pub_->publish(state_msg);

            std_msgs::msg::Bool follow_msg;
            follow_msg.data = (state_ == MissionState::FOLLOWING);
            follow_enabled_pub_->publish(follow_msg);
        }

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_valid_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_pos_sub_;


        rclcpp::Publisher<drone_interfaces::msg::MissionState>::SharedPtr mission_state_pub_;           
               
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr follow_enabled_pub_;
                                                        
        rclcpp::TimerBase::SharedPtr timer_;
                                                        
        MissionState state_;
        bool target_valid_;
        double target_pos_x_;
        double target_pos_y_;

  };

  int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<MissionManagerNode>());
    rclcpp::shutdown();
    return 0;
  }
       