#include "cartesian_manager/ros/tf_pose_publisher.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/time.hpp"

namespace ros_cartesian_manager
{
namespace
{
constexpr double kQuaternionEpsilon = 1e-12;

std::chrono::nanoseconds timerPeriod(double publish_rate_hz)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / publish_rate_hz));
}

void requireNonEmpty(const std::string & value, const char *parameter_name)
{
  if (value.empty()) {
    throw std::invalid_argument(std::string(parameter_name) + " must not be empty");
  }
}

bool isFinite(double value)
{
  return std::isfinite(value);
}
}   // namespace

TfPosePublisher::TfPosePublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("tf_pose_publisher", options)
{
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  effector_frame_id_ = declare_parameter<std::string>("effector_frame_id", "tool0");
  output_topic_ = declare_parameter<std::string>("output_topic", "/ee_pose");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);

  requireNonEmpty(base_frame_id_, "base_frame_id");
  requireNonEmpty(effector_frame_id_, "effector_frame_id");
  requireNonEmpty(output_topic_, "output_topic");
  if (!isFinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
    throw std::invalid_argument("publish_rate_hz must be finite and > 0.0");
  }
  const auto publish_period = timerPeriod(publish_rate_hz_);
  if (publish_period <= std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument("publish_rate_hz is too high to produce a valid timer period");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);
  timer_ = create_wall_timer(publish_period, [this]() {publishPose();});
}

void TfPosePublisher::publishPose()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform =
      tf_buffer_->lookupTransform(base_frame_id_, effector_frame_id_, tf2::TimePointZero);
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cannot publish end-effector pose: TF '%s' <- '%s' is unavailable: %s",
                           base_frame_id_.c_str(), effector_frame_id_.c_str(), error.what());
    return;
  }

  const auto & translation = transform.transform.translation;
  const auto & orientation = transform.transform.rotation;
  const double quaternion_norm =
    std::sqrt(orientation.w * orientation.w + orientation.x * orientation.x +
                  orientation.y * orientation.y + orientation.z * orientation.z);
  const bool transform_is_valid =
    isFinite(translation.x) && isFinite(translation.y) && isFinite(translation.z) &&
    isFinite(orientation.w) && isFinite(orientation.x) && isFinite(orientation.y) &&
    isFinite(orientation.z) && isFinite(quaternion_norm) &&
    quaternion_norm > kQuaternionEpsilon;
  if (!transform_is_valid) {
    RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Cannot publish end-effector pose: TF '%s' <- '%s' contains invalid values",
          base_frame_id_.c_str(), effector_frame_id_.c_str());
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = transform.header.stamp;
  pose.header.frame_id = base_frame_id_;
  pose.pose.position.x = translation.x;
  pose.pose.position.y = translation.y;
  pose.pose.position.z = translation.z;
  pose.pose.orientation.w = orientation.w / quaternion_norm;
  pose.pose.orientation.x = orientation.x / quaternion_norm;
  pose.pose.orientation.y = orientation.y / quaternion_norm;
  pose.pose.orientation.z = orientation.z / quaternion_norm;
  pose_publisher_->publish(pose);
}

} // namespace ros_cartesian_manager
