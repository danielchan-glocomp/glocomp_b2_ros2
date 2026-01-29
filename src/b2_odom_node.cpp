#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "unitree_go/msg/time_spec.hpp"
#include "tf2/LinearMath/Quaternion.h"



class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher()
  : Node("b2_odom_publisher")
  {
    // Initialize the odom publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom",10);

    // Lol
    time_sub = this->create_subscription<nav_msgs::msg::Odometry>(
	"dog_odom", 1,
	std::bind(&OdomPublisher::get_time, this, std::placeholders::_1));

    // Grab the odometry
    std::string topic_name = "sportmodestate";
    subscription_ = this->create_subscription<unitree_go::msg::SportModeState>(
      topic_name, 10,
      std::bind(&OdomPublisher::handle_odom, this, std::placeholders::_1));
  }

private:
  void get_time(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    stamp = msg->header.stamp;
  }

  void handle_odom(const std::shared_ptr<unitree_go::msg::SportModeState> msg)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;

    odom.header.frame_id = "odom";
    odom.child_frame_id = "robot_center";

    odom.pose.pose.position.x = msg->position[0];
    odom.pose.pose.position.y = msg->position[1];
    odom.pose.pose.position.z = msg->position[2];

    odom.pose.pose.orientation.x = msg->imu_state.quaternion[1];
    odom.pose.pose.orientation.y = msg->imu_state.quaternion[2];
    odom.pose.pose.orientation.z = msg->imu_state.quaternion[3];
    odom.pose.pose.orientation.w = msg->imu_state.quaternion[0];

    odom.twist.twist.linear.x = msg->velocity[0];
    odom.twist.twist.linear.y = msg->velocity[1];
    odom.twist.twist.linear.z = msg->velocity[2];
    odom.twist.twist.angular.z = msg->yaw_speed;

    // Send the transformation
    odom_pub_->publish(odom);


  }
  builtin_interfaces::msg::Time stamp;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr time_sub;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
