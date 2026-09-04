#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace ros_cartesian_manager
{

  /**
   * @brief Publish an end-effector PoseStamped from the latest TF transform.
   *
   * This adapter deliberately stays separate from CartesianManagerROS. It provides the same
   * /ee_pose interface that a controller can publish directly in the future.
   */
class TfPosePublisher : public rclcpp::Node
{
public:
  explicit TfPosePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publishPose();

  std::string base_frame_id_;
  std::string effector_frame_id_;
  std::string output_topic_;
  double publish_rate_hz_{100.0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace ros_cartesian_manager
