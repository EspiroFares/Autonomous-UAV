#include <chrono>                                              
#include <functional>                                          
#include <memory>                                              
#include <cmath>

#include "rclcpp/rclcpp.hpp"                                   
#include "drone_interfaces/msg/target_state.hpp"
                                                                
using namespace std::chrono_literals; 

class MockTargetNode : public rclcpp::Node {
    public:
        MockTargetNode():
        Node("mock_target_node"),
        t_(0.0)
        {
            target_pub_ = this->create_publisher<drone_interfaces::msg::TargetState>("/target/state", 10);

            timer_ = this->create_wall_timer(100ms, std::bind(&MockTargetNode::Update, this));

            RCLCPP_INFO(this->get_logger(), "mock_target_node started");
        }

    private:
     void Update() {
        drone_interfaces::msg::TargetState msg;

        msg.detected = true;
        msg.confidence = 0.9f;
        msg.yaw_error = static_cast<float>(std::sin(t_)* 0.3);
        msg.distance_estimate = 1.5f;
        msg.bbox_center_x = 0.0f;
        msg.bbox_center_y = 0.0f;
        msg.bbox_width = 0.0f;
        msg.bbox_height = 0.0f;

        t_ += 0.1;

        target_pub_->publish(msg);
     }

       rclcpp::Publisher<drone_interfaces::msg::TargetState>::SharedPtr target_pub_;                                                               
      rclcpp::TimerBase::SharedPtr timer_;                                   
      double t_;                                                             
  };

  int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockTargetNode>());
    rclcpp::shutdown();                                                    
    return 0;
  }


