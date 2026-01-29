#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "unitree_go/msg/time_spec.hpp"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("b2_tf_frame_publisher")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    time_sub = this->create_subscription<nav_msgs::msg::Odometry>(
	"dog_odom", 1,
	std::bind(&FramePublisher::get_time, this, std::placeholders::_1));


    std::string topic_name = "sportmodestate";
    subscription_ = this->create_subscription<unitree_go::msg::SportModeState>(
      topic_name, 10,
      std::bind(&FramePublisher::handle_tf, this, std::placeholders::_1));
  }

private:
  void get_time(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    stamp = msg->header.stamp;
  }

  void handle_tf(const std::shared_ptr<unitree_go::msg::SportModeState> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = stamp;

    t.header.frame_id = "odom";
    t.child_frame_id = "robot_center";


    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->position[0];
    t.transform.translation.y = msg->position[1];
    t.transform.translation.z = msg->position[2];

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    //tf2::Quaternion q;
    //q.setRPY(M_PI, 0, 0);
    t.transform.rotation.x = msg->imu_state.quaternion[1];
    t.transform.rotation.y = msg->imu_state.quaternion[2];
    t.transform.rotation.z = msg->imu_state.quaternion[3];
    t.transform.rotation.w = msg->imu_state.quaternion[0];
    //tf2::Quaternion q_orig(
    //    t.transform.rotation.x,
    //    t.transform.rotation.y,
    //    t.transform.rotation.z,
    //    t.transform.rotation.w
    //);
    // Multiply (tf2 ONLY)
    //tf2::Quaternion q_new = q * q_orig;
    //q_new.normalize();

    // Convert back to geometry_msgs
    //t.transform.rotation.x = q_new.x();
    //t.transform.rotation.y = q_new.y();
    //t.transform.rotation.z = q_new.z();
    //t.transform.rotation.w = q_new.w();


    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    geometry_msgs::msg::TransformStamped t1;

    // Read message content and assign it to
    // corresponding tf variables
    t1.header.stamp = stamp;
    t1.header.frame_id = "robot_center";
    t1.child_frame_id = "rslidar";


    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t1.transform.translation.x = 0;
    t1.transform.translation.y = 0;
    t1.transform.translation.z = 0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q1;
    q1.setRPY(0, 0, 0);
    t1.transform.rotation.x = q1.x();
    t1.transform.rotation.y = q1.y();
    t1.transform.rotation.z = q1.z();
    t1.transform.rotation.w = q1.w();
    // Send the transformation
    tf_broadcaster_->sendTransform(t1);

    geometry_msgs::msg::TransformStamped t2;
    // Read message content and assign it to
    // corresponding tf variables
    t2.header.stamp = stamp;

    t2.header.frame_id = "robot_center";
    t2.child_frame_id = "base_footprint";


    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t2.transform.translation.x = 0;
    t2.transform.translation.y = 0;
    t2.transform.translation.z = -t.transform.translation.z;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q2;
    q2.setRPY(0, 0, 0);
    t2.transform.rotation.x = q2.x();
    t2.transform.rotation.y = q2.y();
    t2.transform.rotation.z = q2.z();
    t2.transform.rotation.w = q2.w();
    // Send the transformation
    tf_broadcaster_->sendTransform(t2);


  }
  builtin_interfaces::msg::Time stamp;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr time_sub;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}
