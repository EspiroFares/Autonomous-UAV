#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp" 
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

    class CameraDriverNode : public rclcpp::Node {
    public:
        CameraDriverNode() : Node("camera_driver_node") {
            pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
            timer_ = this->create_wall_timer(33ms, std::bind(&CameraDriverNode::update, this));

            cap_.open(0);
           if (!cap_.isOpened()) {
              RCLCPP_ERROR(this->get_logger(), "Could not open camera");
          } else {
              RCLCPP_INFO(this->get_logger(), "camera_driver_node started");
          }
      }

    private:
        void update() {
            cv::Mat frame;
            if (!cap_.read(frame)) {
                RCLCPP_WARN(this->get_logger(), "Failed to read frame");
                return;
            }
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            pub_->publish(*msg);
        }
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        cv::VideoCapture cap_;
    };
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraDriverNode>());
    rclcpp::shutdown();
    return 0;
}
