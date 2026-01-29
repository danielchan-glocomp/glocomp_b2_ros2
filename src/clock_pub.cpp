#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class ClockPublisherNode : public rclcpp::Node
{
public:
    ClockPublisherNode() : Node("clock_publisher_node")
    {
        // Publisher for /clock
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Subscriber to dog_odom
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dog_odom",
            10,
            std::bind(&ClockPublisherNode::topic_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "ClockPublisherNode started.");
    }

private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        (void)msg;  // We don't need the message content, just trigger on reception

        // Get current ROS time
        auto now = msg->header.stamp;

        // Publish it as /clock
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = now;
        clock_pub_->publish(clock_msg);

        RCLCPP_INFO(this->get_logger(), "Published /clock: %u.%u", now.sec, now.nanosec);
    }

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClockPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

