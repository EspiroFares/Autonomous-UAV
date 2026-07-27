// Person tracker: smooths the raw per-frame detections with an EMA so the
// downstream distance and bearing estimates don't jitter. Publishes an invalid
// track as soon as detection drops so the mission logic can react.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "drone_interfaces/msg/detection.hpp"
#include "drone_interfaces/msg/track.hpp"

class PersonTrackerNode : public rclcpp::Node {
public:
    PersonTrackerNode()
    : Node("person_tracker_node"),
      track_id_(0),
      has_track_(false),
      has_shoulder_measurement_(false),
      has_torso_measurement_(false),
      cx_(0.0f),
      cy_(0.0f),
      width_(0.0f),
      height_(0.0f),
      shoulder_width_px_(0.0f),
      torso_height_px_(0.0f),
      alpha_(0.6f)
    {
        sub_ = this->create_subscription<drone_interfaces::msg::Detection>(
            "/target/detections",
            10,
            std::bind(
                &PersonTrackerNode::on_detection,
                this,
                std::placeholders::_1));

        pub_ = this->create_publisher<drone_interfaces::msg::Track>(
            "/target/track",
            10);

        RCLCPP_INFO(this->get_logger(), "person_tracker_node started");
    }

private:
    // Exponential moving average. Seeds from the first measurement, then blends
    // each new one against the running estimate.
    float filter_measurement(
        const float measurement,
        const float previous,
        const bool has_previous) const
    {
        if (!has_previous) {
            return measurement;
        }

        return alpha_ * measurement + (1.0f - alpha_) * previous;
    }

    void on_detection(
        const drone_interfaces::msg::Detection::SharedPtr msg)
    {
        drone_interfaces::msg::Track track;
        track.header = msg->header;
        track.track_id = track_id_;

        if (!msg->detected) {
            has_track_ = false;
            has_shoulder_measurement_ = false;
            has_torso_measurement_ = false;

            track.valid = false;
            pub_->publish(track);
            return;
        }

        if (!has_track_) {
            // Fresh acquisition: seed straight from this detection instead of
            // blending in leftover state from an old track.
            ++track_id_;
            track.track_id = track_id_;

            cx_ = msg->bbox_center_x;
            cy_ = msg->bbox_center_y;
            width_ = 0.0f;
            height_ = 0.0f;
            shoulder_width_px_ = 0.0f;
            torso_height_px_ = 0.0f;
            has_shoulder_measurement_ = false;
            has_torso_measurement_ = false;
            has_track_ = true;
        } else {
            cx_ =
                alpha_ * msg->bbox_center_x +
                (1.0f - alpha_) * cx_;
            cy_ =
                alpha_ * msg->bbox_center_y +
                (1.0f - alpha_) * cy_;
        }

        if (msg->shoulder_valid) {
            width_ = filter_measurement(
                msg->bbox_width,
                width_,
                has_shoulder_measurement_);
            shoulder_width_px_ = filter_measurement(
                msg->shoulder_width_px,
                shoulder_width_px_,
                has_shoulder_measurement_);
            has_shoulder_measurement_ = true;
        }

        if (msg->torso_valid) {
            height_ = filter_measurement(
                msg->bbox_height,
                height_,
                has_torso_measurement_);
            torso_height_px_ = filter_measurement(
                msg->torso_height_px,
                torso_height_px_,
                has_torso_measurement_);
            has_torso_measurement_ = true;
        }

        track.valid =
            has_shoulder_measurement_ || has_torso_measurement_;
        track.track_id = track_id_;
        track.center_x = cx_;
        track.center_y = cy_;
        track.width = width_;
        track.height = height_;
        track.velocity_x = 0.0f;
        track.velocity_y = 0.0f;
        track.shoulder_width_px = shoulder_width_px_;
        track.torso_height_px = torso_height_px_;
        track.shoulder_valid =
            msg->shoulder_valid && has_shoulder_measurement_;
        track.torso_valid =
            msg->torso_valid && has_torso_measurement_;

        pub_->publish(track);
    }

    rclcpp::Subscription<
        drone_interfaces::msg::Detection>::SharedPtr sub_;
    rclcpp::Publisher<
        drone_interfaces::msg::Track>::SharedPtr pub_;

    uint32_t track_id_;
    bool has_track_;
    bool has_shoulder_measurement_;
    bool has_torso_measurement_;
    float cx_;
    float cy_;
    float width_;
    float height_;
    float shoulder_width_px_;
    float torso_height_px_;
    float alpha_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PersonTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
