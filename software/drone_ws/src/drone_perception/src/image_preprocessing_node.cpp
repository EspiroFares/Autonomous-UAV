// Resizes the camera feed to a fixed 640x480 and republishes it. Also runs a
// self-healing watchdog: if no frames arrive within 5 s it exits so the launch
// loop respawns it and re-runs camera discovery.

#include <functional>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class ImagePreprocessingNode : public rclcpp::Node {
public:
    ImagePreprocessingNode() : Node("image_preprocessing_node") {
        auto qos = rclcpp::QoS(1).best_effort();

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", qos,
            std::bind(&ImagePreprocessingNode::on_image, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/image_preprocessed", qos);

        start_time_ = this->now();
        watchdog_ = this->create_wall_timer(1s, std::bind(&ImagePreprocessingNode::check_alive, this));


        RCLCPP_INFO(this->get_logger(), "image_preprocessing_node started");
    }

private:

void check_alive() {
        if (!got_frame_ && (this->now() - start_time_).seconds() > 5.0) {
            RCLCPP_ERROR(this->get_logger(),
                "No camera frames after 5s - exiting for restart");
            rclcpp::shutdown();   // launch loop respawns us -> fresh camera discovery
        }
    }
void on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    got_frame_ = true;
    auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat resized;
    cv::resize(cv_ptr->image, resized, cv::Size(640, 480));
    auto out = cv_bridge::CvImage(msg->header, "bgr8", resized).toImageMsg();
    pub_->publish(*out); 
}
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; 
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_; 
    rclcpp::Time start_time_;
    bool got_frame_ = false;
    rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<ImagePreprocessingNode>()); 
    rclcpp::shutdown();
    return 0;
}