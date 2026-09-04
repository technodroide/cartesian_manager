#include <memory>

#include "cartesian_manager/ros/tf_pose_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros_cartesian_manager::TfPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
