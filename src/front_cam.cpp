#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RtspPublisher : public rclcpp::Node {
public:
    RtspPublisher() : Node("front_cam_publisher") {
        cap_.open("rtsp://192.168.123.161:8551/front_video", cv::CAP_FFMPEG);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "RTSP stream opened successfully");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&RtspPublisher::publish_frame, this)
        );
    }

    void init_image_transport() {
        it_pub_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        pub_ = it_pub_->advertise("front_camera/image_raw", 10);
    }

private:
    void publish_frame() {
        cv::Mat frame;
        if (!cap_.read(frame)){
	    RCLCPP_INFO(this->get_logger(), "Frame missing");
	    return;
	}

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        pub_.publish(msg);  // .publish because Publisher is not a pointer
    }

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<image_transport::ImageTransport> it_pub_;
    image_transport::Publisher pub_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RtspPublisher>();

    // Now the node is fully managed by shared_ptr
    node->init_image_transport();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

