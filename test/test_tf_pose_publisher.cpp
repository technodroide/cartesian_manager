#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "cartesian_manager/ros/cartesian_manager.hpp"
#include "cartesian_manager/ros/tf_pose_publisher.hpp"
#include "extender_msgs/msg/cartesian_velocity_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace
{
using namespace std::chrono_literals;

constexpr double kPi = 3.14159265358979323846;

bool spinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & condition,
  const std::function<void()> & publish_inputs,
  std::chrono::milliseconds timeout = 2s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!condition() && std::chrono::steady_clock::now() < deadline) {
    publish_inputs();
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  return condition();
}

rclcpp::NodeOptions adapterOptions(
  const std::string & base_frame,
  const std::string & effector_frame,
  const std::string & output_topic,
  double publish_rate_hz = 200.0)
{
  return rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("base_frame_id", base_frame),
      rclcpp::Parameter("effector_frame_id", effector_frame),
      rclcpp::Parameter("output_topic", output_topic),
      rclcpp::Parameter("publish_rate_hz", publish_rate_hz)});
}

rclcpp::NodeOptions managerOptions(
  const std::string & prefix,
  const std::string & base_frame)
{
  return rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("update_rate_hz", 200.0),
      rclcpp::Parameter("output_frame_id", base_frame),
      rclcpp::Parameter("default_input_frame_id", base_frame),
      rclcpp::Parameter("topics.joystick_command", prefix + "/joystick"),
      rclcpp::Parameter("topics.visual_servoing_command", prefix + "/visual"),
      rclcpp::Parameter("topics.mode_request", prefix + "/mode"),
      rclcpp::Parameter("topics.ee_pose", prefix + "/pose"),
      rclcpp::Parameter("topics.ee_vel", prefix + "/velocity"),
      rclcpp::Parameter("topics.ee_jac", prefix + "/jacobian"),
      rclcpp::Parameter("topics.joint_states", prefix + "/joints"),
      rclcpp::Parameter("topics.joint_target_command", prefix + "/joint_target"),
      rclcpp::Parameter("topics.output_command", prefix + "/output"),
      rclcpp::Parameter("inputs.sources", std::vector<std::string>{"joystick"}),
      rclcpp::Parameter("inputs.joystick.timeout_sec", 0.5)});
}

class TfPosePublisherTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("ROS_LOG_DIR", "/tmp", 1);
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};
} // namespace

TEST_F(TfPosePublisherTest, PublishesNothingWithoutTransform)
{
  auto adapter = std::make_shared<ros_cartesian_manager::TfPosePublisher>(
      adapterOptions("missing_base", "missing_tool", "/test_tf_pose_missing/pose"));
  auto client = std::make_shared<rclcpp::Node>("tf_pose_missing_test_client");

  bool pose_received = false;
  auto pose_sub = client->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/test_tf_pose_missing/pose", 10,
    [&pose_received](const geometry_msgs::msg::PoseStamped &) {pose_received = true;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter);
  executor.add_node(client);
  spinUntil(executor, []() {return false;}, []() {}, 200ms);
  EXPECT_FALSE(pose_received);
  (void)pose_sub;
}

TEST_F(TfPosePublisherTest, PublishesConfiguredTransformAsNormalizedPose)
{
  const std::string base_frame = "tf_pose_custom_base";
  const std::string effector_frame = "tf_pose_custom_tool";
  const std::string output_topic = "/test_tf_pose_custom/output";
  auto adapter = std::make_shared<ros_cartesian_manager::TfPosePublisher>(
      adapterOptions(base_frame, effector_frame, output_topic));
  auto client = std::make_shared<rclcpp::Node>("tf_pose_custom_test_client");
  tf2_ros::TransformBroadcaster broadcaster(*client);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = client->now();
  transform.header.frame_id = base_frame;
  transform.child_frame_id = effector_frame;
  transform.transform.translation.x = 0.4;
  transform.transform.translation.y = -0.2;
  transform.transform.translation.z = 0.7;
  const double scaled_half_sqrt = std::sqrt(0.5) * 1.001;
  transform.transform.rotation.w = scaled_half_sqrt;
  transform.transform.rotation.z = scaled_half_sqrt;

  bool received_expected = false;
  auto pose_sub = client->create_subscription<geometry_msgs::msg::PoseStamped>(
      output_topic, 10,
    [&received_expected, &transform, &base_frame](const geometry_msgs::msg::PoseStamped & pose) {
      received_expected =
      pose.header.frame_id == base_frame &&
      pose.header.stamp.sec == transform.header.stamp.sec &&
      pose.header.stamp.nanosec == transform.header.stamp.nanosec &&
      std::abs(pose.pose.position.x - 0.4) < 1e-9 &&
      std::abs(pose.pose.position.y + 0.2) < 1e-9 &&
      std::abs(pose.pose.position.z - 0.7) < 1e-9 &&
      std::abs(pose.pose.orientation.w - std::sqrt(0.5)) < 1e-9 &&
      std::abs(pose.pose.orientation.x) < 1e-9 &&
      std::abs(pose.pose.orientation.y) < 1e-9 &&
      std::abs(pose.pose.orientation.z - std::sqrt(0.5)) < 1e-9;
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter);
  executor.add_node(client);
  EXPECT_TRUE(spinUntil(executor, [&received_expected]() {return received_expected;},
    [&broadcaster, &transform]() {broadcaster.sendTransform(transform);}));
  (void)pose_sub;
}

TEST_F(TfPosePublisherTest, PublishesNothingForInvalidTransform)
{
  const std::string base_frame = "tf_pose_invalid_transform_base";
  const std::string effector_frame = "tf_pose_invalid_transform_tool";
  const std::string output_topic = "/test_tf_pose_invalid_transform/output";
  auto adapter = std::make_shared<ros_cartesian_manager::TfPosePublisher>(
    adapterOptions(base_frame, effector_frame, output_topic));
  auto client = std::make_shared<rclcpp::Node>("tf_pose_invalid_transform_test_client");
  tf2_ros::TransformBroadcaster broadcaster(*client);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = client->now();
  transform.header.frame_id = base_frame;
  transform.child_frame_id = effector_frame;
  transform.transform.rotation.w = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;

  bool pose_received = false;
  auto pose_sub = client->create_subscription<geometry_msgs::msg::PoseStamped>(
    output_topic, 10,
    [&pose_received](const geometry_msgs::msg::PoseStamped &) {pose_received = true;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter);
  executor.add_node(client);
  spinUntil(executor, []() {return false;},
    [&broadcaster, &transform]() {broadcaster.sendTransform(transform);}, 250ms);
  EXPECT_FALSE(pose_received);
  (void)pose_sub;
}

TEST_F(TfPosePublisherTest, RejectsInvalidParameters)
{
  auto options = adapterOptions("", "tool", "/test_tf_pose_invalid/pose");
  EXPECT_THROW(std::make_shared<ros_cartesian_manager::TfPosePublisher>(options),
               std::invalid_argument);

  options = adapterOptions("base", "", "/test_tf_pose_invalid/pose");
  EXPECT_THROW(std::make_shared<ros_cartesian_manager::TfPosePublisher>(options),
               std::invalid_argument);

  options = adapterOptions("base", "tool", "");
  EXPECT_THROW(std::make_shared<ros_cartesian_manager::TfPosePublisher>(options),
               std::invalid_argument);

  options = adapterOptions("base", "tool", "/test_tf_pose_invalid/pose", 0.0);
  EXPECT_THROW(std::make_shared<ros_cartesian_manager::TfPosePublisher>(options),
               std::invalid_argument);

  options = adapterOptions("base", "tool", "/test_tf_pose_invalid/pose",
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(std::make_shared<ros_cartesian_manager::TfPosePublisher>(options),
               std::invalid_argument);

  options = adapterOptions("base", "tool", "/test_tf_pose_invalid/pose",
      std::numeric_limits<double>::max());
  EXPECT_THROW(std::make_shared<ros_cartesian_manager::TfPosePublisher>(options),
               std::invalid_argument);
}

TEST_F(TfPosePublisherTest, AdapterEnablesEffectorFrameJoystickCommands)
{
  const std::string prefix = "/test_tf_pose_manager";
  const std::string base_frame = "tf_pose_manager_base";
  const std::string effector_frame = "tf_pose_manager_tool";
  auto manager = std::make_shared<ros_cartesian_manager::CartesianManagerROS>(
      managerOptions(prefix, base_frame));
  auto adapter = std::make_shared<ros_cartesian_manager::TfPosePublisher>(
      adapterOptions(base_frame, effector_frame, prefix + "/pose"));
  auto client = std::make_shared<rclcpp::Node>("tf_pose_manager_test_client");
  tf2_ros::TransformBroadcaster broadcaster(*client);

  auto command_pub = client->create_publisher<extender_msgs::msg::CartesianVelocityCommand>(
      prefix + "/joystick", 10);
  bool received_nonzero = false;
  bool received_expected = false;
  auto output_sub = client->create_subscription<geometry_msgs::msg::TwistStamped>(
      prefix + "/output", 10,
    [&received_nonzero, &received_expected](const geometry_msgs::msg::TwistStamped & msg) {
      const double magnitude = std::abs(msg.twist.linear.x) + std::abs(msg.twist.angular.x) +
      std::abs(msg.twist.angular.y) + std::abs(msg.twist.angular.z);
      received_nonzero = received_nonzero || magnitude > 1e-9;
      received_expected = received_expected ||
      (std::abs(msg.twist.linear.x - 0.25) < 1e-9 &&
      std::abs(msg.twist.angular.x) < 1e-9 &&
      std::abs(msg.twist.angular.y - 1.0) < 1e-9);
      });

  extender_msgs::msg::CartesianVelocityCommand command;
  command.header.frame_id = base_frame;
  command.orientation_frame_id = command.EFFECTOR_FRAME;
  command.twist.linear.x = 0.25;
  command.twist.angular.x = 1.0;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = client->now();
  transform.header.frame_id = base_frame;
  transform.child_frame_id = effector_frame;
  transform.transform.rotation.w = std::cos(kPi / 4.0);
  transform.transform.rotation.z = std::sin(kPi / 4.0);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(manager);
  executor.add_node(adapter);
  executor.add_node(client);

  spinUntil(executor, []() {return false;},
    [&command_pub, &command]() {command_pub->publish(command);}, 200ms);
  EXPECT_FALSE(received_nonzero);

  EXPECT_TRUE(spinUntil(
      executor, [&received_expected]() {return received_expected;},
      [&broadcaster, &transform, &command_pub, &command]() {
        broadcaster.sendTransform(transform);
        command_pub->publish(command);
      }));
  (void)output_sub;
}
