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
        cv::Mat frame_raw, frame, gray;
        if (!cap_.isOpened()) return;

        cap_ >> frame_raw;
        if (frame_raw.empty()) {
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            return;
        }

         cv::rotate(frame_raw, frame, cv::ROTATE_180);
        // --- Face detection ---

        // Grayscale the image
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray, gray);

        // Search for faces
        std::vector<cv::Rect> faces;
        face_cascade_.detectMultiScale(gray, faces, 1.1, 3, 0, cv::Size(30,30));

        // Draw rectangle around faces
        for (size_t i = 0; i < faces.size(); i++) {
            cv::rectangle(frame, faces[i], cv::Scalar(255,0,0) , 3);
        }

        // save snapshop
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