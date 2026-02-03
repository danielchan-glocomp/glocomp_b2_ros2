#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuRepublisherNode : public rclcpp::Node
{
public:
    ClockPublisherNode() : Node("imu_republisher_node")
    {
        // Publisher for /imu
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

        // Subscriber to dog_imu_raw
        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/dog_imu_raw",
            10,
            std::bind(&ClockPublisherNode::topic_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "LidarRepublisherNode started.");
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        (void)msg;  // We don't need the message content, just trigger on reception
        
        sensor_msgs::msg::Imu repub_imu = msg;
        repub_imu.orientation_covariance = {
                                            0.0004, 0.0,    0.0,
                                            0.0,    0.0004, 0.0,
                                            0.0,    0.0,    0.0006};
        repub_imu.angular_velocity_covariance = {
                                            0.0004, 0.0,    0.0,
                                            0.0,    0.0004, 0.0,
                                            0.0,    0.0,    0.0006};

        repub_imu.linear_acceleration_covariance = {
                                            0.0004, 0.0,    0.0,
                                            0.0,    0.0004, 0.0,
                                            0.0,    0.0,    0.0006};

        // Publish it as /imu
        lidar_pub_->publish(repub_imu);

    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuRepublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


