#include <functional>
#include <memory> 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

class ImagePreprocessingNode : public rclcpp::Node {
public:
    ImagePreprocessingNode() : Node("image_preprocessing_node") {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&ImagePreprocessingNode::on_image, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_preprocessed", 10);

        RCLCPP_INFO(this->get_logger(), "image_preprocessing_node started");
    }

private:
void on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat resized;
    cv::resize(cv_ptr->image, resized, cv::Size(640, 480));
    auto out = cv_bridge::CvImage(msg->header, "bgr8", resized).toImageMsg();
    pub_->publish(*out); 
}
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; 
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_; 
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<ImagePreprocessingNode>()); 
    rclcpp::shutdown();
    return 0;
}