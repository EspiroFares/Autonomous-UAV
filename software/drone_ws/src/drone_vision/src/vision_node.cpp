#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std::chrono_literals;

class VisionNode : public rclcpp::Node {
public:

    VisionNode() : Node("vision_node") {
        // 1. Load mp4 file
        bool video_success = cap_.open("/home/fares/Drone/test.mp4");

        if (!video_success) {
            RCLCPP_ERROR(this->get_logger(), "Video not found");
        } else {
            RCLCPP_INFO(this->get_logger(), "video loaded");
        }
        // 2. Load AI-modell
        bool model_success = face_cascade_.load("/home/fares/Drone/haarcascade_frontalface_default.xml");
        
        if (!model_success) {
            RCLCPP_ERROR(this->get_logger(), "Kunde inte ladda XML-filen! Har du laddat ner den?");
        } else {
            RCLCPP_INFO(this->get_logger(), "AI-modell laddad. Redo att jaga ansikten!");
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        timer_ = this-> create_wall_timer(33ms, std::bind(&VisionNode::timer_callback, this));
    }

private:
    void timer_callback() {
        cv::Mat frame_raw, frame, gray, small_gray;
        if (!cap_.isOpened()) return;

        cap_ >> frame_raw;
        if (frame_raw.empty()) {
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            return;
        }

        //Keep for now only .mp4 
        cv::rotate(frame_raw, frame, cv::ROTATE_180);

        // --- Downscaling for increasing FPS ---
        // Raspberry pi got 4 FPS on 720p, 320p got 14fps
        double scale_factor = 4;

        // Grayscale the image
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::resize(gray, small_gray, cv::Size(), 1.0/scale_factor, 1.0/scale_factor);
        cv::equalizeHist(small_gray, small_gray);


        // --- Face detection ---
        std::vector<cv::Rect> faces;
        face_cascade_.detectMultiScale(small_gray, faces, 1.1, 3, 0, cv::Size(30,30));

        // Draw rectangle around faces
        for (size_t i = 0; i < faces.size(); i++) {
            cv::Rect big_rect;
            big_rect.x = faces[i].x * scale_factor;
            big_rect.y = faces[i].y * scale_factor;
            big_rect.width = faces[i].width * scale_factor;
            big_rect.height = faces[i].height * scale_factor;

            cv::rectangle(frame, big_rect, cv::Scalar(255, 0, 0), 3);
        }

        // save snapshot
        if (faces.size() > 0 && !snapshot_taken_) {
            cv::imwrite("/home/fares/Drone/bevis.jpg", frame);
            RCLCPP_INFO(this->get_logger(), "SNAP! Ansikte hittat och bild sparad till ~/Drone/bevis.jpg");
            snapshot_taken_ = true; 
        }

        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::VideoCapture cap_;
    cv::CascadeClassifier face_cascade_;
    bool snapshot_taken_ = false;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);                     
    rclcpp::spin(std::make_shared<VisionNode>()); 
    rclcpp::shutdown();           
    return 0;
}