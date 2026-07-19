#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "drone_interfaces/msg/track.hpp"
#include "drone_interfaces/msg/target_state.hpp"

class TargetEstimatorNode : public rclcpp::Node {
public:
    TargetEstimatorNode()
    : Node("target_estimator_node"),
      has_distance_baseline_(false),
      has_reacquisition_candidate_(false),
      has_accepted_shoulder_distance_(false)
    {
        sub_ = this->create_subscription<drone_interfaces::msg::Track>(
            "/target/track",
            10,
            std::bind(
                &TargetEstimatorNode::on_track,
                this,
                std::placeholders::_1));

        pub_ = this->create_publisher<drone_interfaces::msg::TargetState>(
            "/target/state",
            10);

        shoulder_scale_ = static_cast<float>(
            this->declare_parameter<double>(
                "shoulder_scale",
                405.014));
        shoulder_offset_ = static_cast<float>(
            this->declare_parameter<double>(
                "shoulder_offset",
                -0.314));
        torso_scale_ = static_cast<float>(
            this->declare_parameter<double>(
                "torso_scale",
                697.526));
        torso_offset_ = static_cast<float>(
            this->declare_parameter<double>(
                "torso_offset",
                -0.239));
        max_shoulder_range_hold_time_ =
            static_cast<float>(
                this->declare_parameter<double>(
                    "max_shoulder_range_hold_time",
                    0.3));
        min_distance_ = static_cast<float>(
            this->declare_parameter<double>(
                "min_distance",
                0.3));
        max_distance_ = static_cast<float>(
            this->declare_parameter<double>(
                "max_distance",
                5.0));
        median_window_size_ = static_cast<std::size_t>(
            std::clamp(
                this->declare_parameter<int>(
                    "measurement_median_window",
                    5),
                std::int64_t{1},
                std::int64_t{9}));

        RCLCPP_INFO(
            this->get_logger(),
            "target_estimator_node started "
            "(shoulder-only range, scale=%.3f offset=%.3f, "
            "range hold=%.2fs; torso is diagnostics only)",
            static_cast<double>(shoulder_scale_),
            static_cast<double>(shoulder_offset_),
            static_cast<double>(max_shoulder_range_hold_time_));
    }

private:
    void on_track(
        const drone_interfaces::msg::Track::SharedPtr msg)
    {
        drone_interfaces::msg::TargetState target;
        target.stamp = msg->header.stamp;
        target.bbox_center_x = msg->center_x;
        target.bbox_center_y = msg->center_y;
        target.bbox_width = msg->width;
        target.bbox_height = msg->height;

        if (!msg->valid) {
            ResetDistanceState();
            PublishInvalidTarget(target);
            return;
        }

        const float yaw_error = (msg->center_x - 0.5f) * 2.0f;

        const bool center_valid =
            std::isfinite(msg->center_x) &&
            std::isfinite(msg->center_y) &&
            std::isfinite(yaw_error) &&
            msg->center_x >= 0.0f &&
            msg->center_x <= 1.0f &&
            msg->center_y >= 0.0f &&
            msg->center_y <= 1.0f;

        const bool shoulder_pixels_valid =
            msg->shoulder_valid &&
            std::isfinite(msg->shoulder_width_px) &&
            msg->shoulder_width_px >= kMinShoulderWidthPx;
        const bool torso_pixels_valid =
            msg->torso_valid &&
            std::isfinite(msg->torso_height_px) &&
            msg->torso_height_px >= kMinTorsoHeightPx;

        float filtered_shoulder_width_px = 0.0f;
        if (shoulder_pixels_valid) {
            filtered_shoulder_width_px = UpdateMedianWindow(
                shoulder_pixel_window_,
                msg->shoulder_width_px);
        } else {
            shoulder_pixel_window_.clear();
        }

        float filtered_torso_height_px = 0.0f;
        if (torso_pixels_valid) {
            filtered_torso_height_px = UpdateMedianWindow(
                torso_pixel_window_,
                msg->torso_height_px);
        } else {
            torso_pixel_window_.clear();
        }

        const float shoulder_distance =
            shoulder_scale_ /
            (filtered_shoulder_width_px + 1e-6f) +
            shoulder_offset_;
        const float torso_distance =
            torso_scale_ /
            (filtered_torso_height_px + 1e-6f) +
            torso_offset_;

        const bool shoulder_distance_valid =
            shoulder_pixels_valid &&
            std::isfinite(shoulder_distance) &&
            shoulder_distance >= min_distance_ &&
            shoulder_distance <= max_distance_;

        const bool torso_distance_valid =
            torso_pixels_valid &&
            std::isfinite(torso_distance) &&
            torso_distance >= min_distance_ &&
            torso_distance <= max_distance_;

        target.shoulder_distance_estimate =
            shoulder_distance_valid ? shoulder_distance : 0.0f;
        target.torso_distance_estimate =
            torso_distance_valid ? torso_distance : 0.0f;
        target.shoulder_distance_valid = shoulder_distance_valid;
        target.torso_distance_valid = torso_distance_valid;

        if (!center_valid) {
            ResetDistanceState();
            PublishInvalidTarget(target);
            return;
        }

        const auto sample_time =
            std::chrono::steady_clock::now();

        float distance = 0.0f;
        float measurement_confidence = 0.0f;
        bool using_current_shoulder_distance = false;

        if (shoulder_distance_valid) {
            distance = shoulder_distance;
            measurement_confidence = 0.90f;
            using_current_shoulder_distance = true;
        } else if (
            torso_distance_valid &&
            has_accepted_shoulder_distance_)
        {
            const float shoulder_range_age =
                std::chrono::duration<float>(
                    sample_time -
                    last_accepted_shoulder_time_).count();

            if (shoulder_range_age >= 0.0f &&
                shoulder_range_age <=
                    max_shoulder_range_hold_time_)
            {
                // Torso preserves tracking and bearing only. It never
                // supplies control range; briefly hold the last validated
                // shoulder range while the shoulder measurement recovers.
                distance = last_accepted_shoulder_distance_;
                measurement_confidence = 0.60f;
            }
        }

        if (measurement_confidence <= 0.0f ||
            !std::isfinite(distance) ||
            distance < min_distance_ ||
            distance > max_distance_)
        {
            ResetDistanceState();
            PublishInvalidTarget(target);
            return;
        }

        if (!has_distance_baseline_) {
            if (!has_reacquisition_candidate_) {
                StoreReacquisitionCandidate(distance, sample_time);
                PublishRejectedMeasurement(
                    target,
                    yaw_error,
                    distance);
                return;
            }

            const float candidate_age =
                std::chrono::duration<float>(
                    sample_time -
                    reacquisition_candidate_time_).count();
            const float candidate_difference =
                std::abs(
                    distance -
                    reacquisition_candidate_distance_);

            const bool candidate_confirmed =
                candidate_age > 0.0f &&
                candidate_age <= kMaxReacquisitionGap &&
                candidate_difference <= kReacquisitionTolerance;

            if (!candidate_confirmed) {
                StoreReacquisitionCandidate(distance, sample_time);
                PublishRejectedMeasurement(
                    target,
                    yaw_error,
                    distance);
                return;
            }

            has_distance_baseline_ = true;
            has_reacquisition_candidate_ = false;
            last_accepted_distance_ = distance;
            last_sample_time_ = sample_time;
        } else {
            const float sample_dt =
                std::chrono::duration<float>(
                    sample_time -
                    last_sample_time_).count();

            if (sample_dt <= 0.0f ||
                sample_dt > kMaxSampleGap)
            {
                has_distance_baseline_ = false;
                StoreReacquisitionCandidate(
                    distance,
                    sample_time);
                PublishRejectedMeasurement(
                    target,
                    yaw_error,
                    distance);
                return;
            }

            last_sample_time_ = sample_time;

            const float distance_jump =
                std::abs(
                    distance -
                    last_accepted_distance_);
            const float implied_speed =
                distance_jump / sample_dt;

            if (distance_jump > kMaxDistanceJump &&
                implied_speed > kMaxDistanceRate)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Rejected target jump: %.2f m at %.2f m/s",
                    static_cast<double>(distance_jump),
                    static_cast<double>(implied_speed));

                PublishRejectedMeasurement(
                    target,
                    yaw_error,
                    distance);
                return;
            }

            last_accepted_distance_ = distance;
        }

        if (using_current_shoulder_distance) {
            has_accepted_shoulder_distance_ = true;
            last_accepted_shoulder_distance_ = distance;
            last_accepted_shoulder_time_ = sample_time;
        }

        target.detected = true;
        target.confidence = measurement_confidence;
        target.yaw_error = yaw_error;
        target.distance_estimate = distance;
        pub_->publish(target);
    }

    using SteadyTime =
        std::chrono::steady_clock::time_point;

    float UpdateMedianWindow(
        std::deque<float> & window,
        const float sample)
    {
        window.push_back(sample);
        while (window.size() > median_window_size_) {
            window.pop_front();
        }

        std::vector<float> sorted(window.begin(), window.end());
        std::sort(sorted.begin(), sorted.end());

        const std::size_t middle = sorted.size() / 2;
        if (sorted.size() % 2 == 0) {
            return 0.5f * (
                sorted[middle - 1] +
                sorted[middle]);
        }

        return sorted[middle];
    }

    void ResetDistanceState()
    {
        has_distance_baseline_ = false;
        has_reacquisition_candidate_ = false;
        has_accepted_shoulder_distance_ = false;
        shoulder_pixel_window_.clear();
        torso_pixel_window_.clear();
    }

    void PublishInvalidTarget(
        drone_interfaces::msg::TargetState & target)
    {
        target.detected = false;
        target.confidence = 0.0f;
        target.yaw_error = 0.0f;
        target.distance_estimate = 0.0f;
        pub_->publish(target);
    }

    void StoreReacquisitionCandidate(
        const float distance,
        const SteadyTime & sample_time)
    {
        has_reacquisition_candidate_ = true;
        reacquisition_candidate_distance_ = distance;
        reacquisition_candidate_time_ = sample_time;
    }

    void PublishRejectedMeasurement(
        drone_interfaces::msg::TargetState & target,
        const float yaw_error,
        const float distance)
    {
        target.detected = true;
        target.confidence = kRejectedConfidence;
        target.yaw_error = yaw_error;
        target.distance_estimate = distance;
        pub_->publish(target);
    }

    static constexpr float kMinShoulderWidthPx = 35.0f;
    static constexpr float kMinTorsoHeightPx = 30.0f;
    static constexpr float kMaxDistanceJump = 1.0f;
    static constexpr float kMaxDistanceRate = 4.0f;
    static constexpr float kMaxSampleGap = 0.75f;
    static constexpr float kMaxReacquisitionGap = 1.0f;
    static constexpr float kReacquisitionTolerance = 0.5f;
    static constexpr float kRejectedConfidence = 0.3f;

    rclcpp::Subscription<
        drone_interfaces::msg::Track>::SharedPtr sub_;
    rclcpp::Publisher<
        drone_interfaces::msg::TargetState>::SharedPtr pub_;

    bool has_distance_baseline_;
    bool has_reacquisition_candidate_;
    bool has_accepted_shoulder_distance_;
    float last_accepted_distance_;
    float last_accepted_shoulder_distance_;
    float reacquisition_candidate_distance_;
    SteadyTime last_sample_time_;
    SteadyTime reacquisition_candidate_time_;
    SteadyTime last_accepted_shoulder_time_;

    float shoulder_scale_;
    float shoulder_offset_;
    float torso_scale_;
    float torso_offset_;
    float max_shoulder_range_hold_time_;
    float min_distance_;
    float max_distance_;
    std::size_t median_window_size_;
    std::deque<float> shoulder_pixel_window_;
    std::deque<float> torso_pixel_window_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<TargetEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
