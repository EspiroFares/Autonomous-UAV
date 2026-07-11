#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "drone_interfaces/msg/detection.hpp"
#include "drone_interfaces/msg/track.hpp"



class PersonTrackerNode : public rclcpp::Node {
public:
    PersonTrackerNode() : Node("person_tracker_node"),
    track_id_(0),
    has_track_(false),
    cx_(0.0f), cy_(0.0f),
    width_(0.0f), height_(0.0f),
    alpha_(0.6f)
    {
        sub_ = this->create_subscription<drone_interfaces::msg::Detection>("/target/detections", 10, std::bind(&PersonTrackerNode::on_detection, this, std::placeholders::_1));
    
    pub_ = this->create_publisher<drone_interfaces::msg::Track>("/target/track", 10);

    RCLCPP_INFO(this->get_logger(), "person_tracker_node started");
    }

private:
    void on_detection(const drone_interfaces::msg::Detection::SharedPtr msg) {
        drone_interfaces::msg::Track track;
        track.header = msg->header;

        if (!msg->detected) {
            has_track_ = false;

            track.valid = false;
            track.track_id = track_id_;
            pub_->publish(track);
            return;
        }

        if (!has_track_) {
            ++track_id_;

            cx_ = msg->bbox_center_x;
            cy_ = msg->bbox_center_y;
            width_ = msg->bbox_width;
            height_ = msg->bbox_height;

            has_track_ = true;
        } else {
            cx_ = alpha_ * msg->bbox_center_x + (1.0f - alpha_) * cx_;

            cy_ =alpha_ * msg->bbox_center_y + (1.0f - alpha_) * cy_;

            width_ =alpha_ * msg->bbox_width + (1.0f - alpha_) * width_;

            height_ =alpha_ * msg->bbox_height + (1.0f - alpha_) * height_;
        }

        track.valid = true;
        track.track_id = track_id_;
        track.center_x = cx_;
        track.center_y = cy_;
        track.width = width_;
        track.height = height_;
        track.velocity_x = 0.0f;
        track.velocity_y = 0.0f;

        pub_->publish(track);

    }
    rclcpp::Subscription<drone_interfaces::msg::Detection>::SharedPtr sub_;
    rclcpp::Publisher<drone_interfaces::msg::Track>::SharedPtr pub_;

    uint32_t track_id_;
    bool has_track_;
    float cx_, cy_, width_, height_;
    float alpha_;
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PersonTrackerNode>());
    rclcpp::shutdown(); 
    return 0;
}

