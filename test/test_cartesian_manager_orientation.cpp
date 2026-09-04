#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "cartesian_manager/ros/cartesian_manager.hpp"
#include "extender_msgs/msg/cartesian_velocity_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
  using namespace std::chrono_literals;

  constexpr double kPi = 3.14159265358979323846;

  rclcpp::NodeOptions managerOptions(const std::string &topic_prefix)
  {
    return rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("update_rate_hz", 200.0),
         rclcpp::Parameter("topics.joystick_command", topic_prefix + "/joystick"),
         rclcpp::Parameter("topics.visual_servoing_command", topic_prefix + "/visual"),
         rclcpp::Parameter("topics.mode_request", topic_prefix + "/mode"),
         rclcpp::Parameter("topics.ee_pose", topic_prefix + "/pose"),
         rclcpp::Parameter("topics.ee_vel", topic_prefix + "/velocity"),
         rclcpp::Parameter("topics.ee_jac", topic_prefix + "/jacobian"),
         rclcpp::Parameter("topics.joint_states", topic_prefix + "/joints"),
         rclcpp::Parameter("topics.joint_target_command", topic_prefix + "/joint_target"),
         rclcpp::Parameter("topics.output_command", topic_prefix + "/output"),
         rclcpp::Parameter("inputs.sources", std::vector<std::string>{"joystick"}),
         rclcpp::Parameter("inputs.joystick.timeout_sec", 0.5)});
  }

  bool spinUntil(rclcpp::executors::SingleThreadedExecutor &executor,
                 const std::function<bool()> &condition,
                 const std::function<void()> &publish_inputs,
                 std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!condition() && std::chrono::steady_clock::now() < deadline)
    {
      publish_inputs();
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    return condition();
  }

  class CartesianManagerOrientationTest : public ::testing::Test
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

TEST_F(CartesianManagerOrientationTest, BaseFrameDoesNotRequirePose)
{
  const std::string prefix = "/test_cartesian_manager_base";
  auto manager =
      std::make_shared<ros_cartesian_manager::CartesianManagerROS>(managerOptions(prefix));
  auto client = std::make_shared<rclcpp::Node>("cartesian_manager_base_test_client");
  auto command_pub = client->create_publisher<extender_msgs::msg::CartesianVelocityCommand>(
      prefix + "/joystick", 10);

  bool received_expected = false;
  auto output_sub = client->create_subscription<geometry_msgs::msg::TwistStamped>(
      prefix + "/output", 10, [&received_expected](const geometry_msgs::msg::TwistStamped &msg) {
        received_expected = std::abs(msg.twist.linear.x - 0.25) < 1e-9 &&
                            std::abs(msg.twist.angular.z + 0.75) < 1e-9;
      });

  extender_msgs::msg::CartesianVelocityCommand command;
  command.header.frame_id = "base_link";
  command.orientation_frame_id = command.BASE_FRAME;
  command.twist.linear.x = 0.25;
  command.twist.angular.z = -0.75;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(manager);
  executor.add_node(client);
  EXPECT_TRUE(spinUntil(executor, [&received_expected]() { return received_expected; },
                        [&command_pub, &command]() { command_pub->publish(command); }));
  (void)output_sub;
}

TEST_F(CartesianManagerOrientationTest, EffectorFrameWaitsForPoseAndRotatesOnlyAngularPart)
{
  const std::string prefix = "/test_cartesian_manager_effector";
  auto manager =
      std::make_shared<ros_cartesian_manager::CartesianManagerROS>(managerOptions(prefix));
  auto client = std::make_shared<rclcpp::Node>("cartesian_manager_effector_test_client");
  auto command_pub = client->create_publisher<extender_msgs::msg::CartesianVelocityCommand>(
      prefix + "/joystick", 10);
  auto pose_pub =
      client->create_publisher<geometry_msgs::msg::PoseStamped>(prefix + "/pose", 10);

  bool received_nonzero = false;
  bool received_expected = false;
  bool watch_for_zero = false;
  bool received_zero_after_invalid_pose = false;
  auto output_sub = client->create_subscription<geometry_msgs::msg::TwistStamped>(
      prefix + "/output", 10,
      [&received_nonzero, &received_expected, &watch_for_zero,
       &received_zero_after_invalid_pose](const geometry_msgs::msg::TwistStamped &msg) {
        const double magnitude = std::abs(msg.twist.linear.x) + std::abs(msg.twist.angular.x) +
                                 std::abs(msg.twist.angular.y) + std::abs(msg.twist.angular.z);
        received_nonzero = received_nonzero || magnitude > 1e-9;
        if (watch_for_zero && magnitude <= 1e-9)
        {
          received_zero_after_invalid_pose = true;
        }
        received_expected = received_expected ||
                            (std::abs(msg.twist.linear.x - 0.25) < 1e-9 &&
                             std::abs(msg.twist.angular.x) < 1e-9 &&
                             std::abs(msg.twist.angular.y - 1.0) < 1e-9);
      });

  extender_msgs::msg::CartesianVelocityCommand command;
  command.header.frame_id = "base_link";
  command.orientation_frame_id = command.EFFECTOR_FRAME;
  command.twist.linear.x = 0.25;
  command.twist.angular.x = 1.0;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.orientation.w = std::cos(kPi / 4.0);
  pose.pose.orientation.z = std::sin(kPi / 4.0);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(manager);
  executor.add_node(client);

  spinUntil(executor, []() { return false; },
            [&command_pub, &command]() { command_pub->publish(command); }, 200ms);
  EXPECT_FALSE(received_nonzero);

  EXPECT_TRUE(spinUntil(
      executor, [&received_expected]() { return received_expected; },
      [&command_pub, &pose_pub, &command, &pose]() {
        pose_pub->publish(pose);
        command_pub->publish(command);
      }));

  geometry_msgs::msg::PoseStamped invalid_pose;
  invalid_pose.header.frame_id = "base_link";
  watch_for_zero = true;
  EXPECT_TRUE(spinUntil(
      executor, [&received_zero_after_invalid_pose]() { return received_zero_after_invalid_pose; },
      [&pose_pub, &invalid_pose]() { pose_pub->publish(invalid_pose); }));
  (void)output_sub;
}

TEST_F(CartesianManagerOrientationTest, RejectsInvalidConeConfiguration)
{
  auto options = managerOptions("/test_cartesian_manager_invalid_cone");
  options.parameter_overrides().emplace_back("hybrid_frame_cone_angle_deg", 90.0);
  EXPECT_THROW(std::make_shared<ros_cartesian_manager::CartesianManagerROS>(options),
               std::invalid_argument);
}

TEST_F(CartesianManagerOrientationTest, RejectsInvalidSelectorAndPhysicalFrame)
{
  const std::string prefix = "/test_cartesian_manager_invalid_command";
  auto manager =
      std::make_shared<ros_cartesian_manager::CartesianManagerROS>(managerOptions(prefix));
  auto client = std::make_shared<rclcpp::Node>("cartesian_manager_invalid_command_test_client");
  auto command_pub = client->create_publisher<extender_msgs::msg::CartesianVelocityCommand>(
      prefix + "/joystick", 10);

  bool received_nonzero = false;
  auto output_sub = client->create_subscription<geometry_msgs::msg::TwistStamped>(
      prefix + "/output", 10,
      [&received_nonzero](const geometry_msgs::msg::TwistStamped &msg) {
        const double magnitude = std::abs(msg.twist.linear.x) + std::abs(msg.twist.linear.y) +
                                 std::abs(msg.twist.linear.z) + std::abs(msg.twist.angular.x) +
                                 std::abs(msg.twist.angular.y) + std::abs(msg.twist.angular.z);
        received_nonzero = received_nonzero || magnitude > 1e-9;
      });

  extender_msgs::msg::CartesianVelocityCommand command;
  command.header.frame_id = "base_link";
  command.orientation_frame_id = "ee_frame";
  command.twist.linear.x = 1.0;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(manager);
  executor.add_node(client);

  spinUntil(executor, []() { return false; },
            [&command_pub, &command]() { command_pub->publish(command); }, 150ms);
  EXPECT_FALSE(received_nonzero);

  command.header.frame_id = "tool0";
  command.orientation_frame_id = command.BASE_FRAME;
  spinUntil(executor, []() { return false; },
            [&command_pub, &command]() { command_pub->publish(command); }, 150ms);
  EXPECT_FALSE(received_nonzero);
  (void)output_sub;
}
