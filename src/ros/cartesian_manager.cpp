#include "cartesian_manager/ros/cartesian_manager.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "extender_msgs/msg/cartesian_velocity_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros_cartesian_manager
{
  namespace
  {
    constexpr const char *kOutputCommandPublisher = "output_command";
    constexpr const char *kJointTargetCommandPublisher = "joint_target_command";
    constexpr const char *kBehaviourPassthroughMode = "behaviour/passthrough";
    constexpr const char *kJointTargetModePrefix = "behaviour/joint_target/";
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kQuaternionEpsilon = 1e-12;

    std::chrono::nanoseconds timerPeriod(double update_rate_hz)
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / update_rate_hz));
    }

    bool inputSourcesEqual(const std::vector<InputConfig> &lhs, const std::vector<InputConfig> &rhs)
    {
      if (lhs.size() != rhs.size())
      {
        return false;
      }

      for (std::size_t index = 0; index < lhs.size(); ++index)
      {
        if (lhs[index].source != rhs[index].source)
        {
          return false;
        }
      }

      return true;
    }

    bool inputConfigsEqual(const std::vector<InputConfig> &lhs, const std::vector<InputConfig> &rhs)
    {
      if (!inputSourcesEqual(lhs, rhs))
      {
        return false;
      }

      for (std::size_t index = 0; index < lhs.size(); ++index)
      {
        if (lhs[index].timeout_sec != rhs[index].timeout_sec ||
            lhs[index].enabled != rhs[index].enabled)
        {
          return false;
        }
      }

      return true;
    }

    bool usedTopicsEqual(const TopicConfig &lhs, const TopicConfig &rhs)
    {
      return lhs.joystick_command == rhs.joystick_command &&
             lhs.visual_servoing_command == rhs.visual_servoing_command &&
             lhs.mode_request == rhs.mode_request && lhs.output_command == rhs.output_command &&
             lhs.joint_target_command == rhs.joint_target_command &&
             lhs.ee_pose == rhs.ee_pose && lhs.ee_vel == rhs.ee_vel && lhs.ee_jac == rhs.ee_jac &&
             lhs.joint_states == rhs.joint_states;
    }

    bool jointTargetsEqual(const manager_core::JointTargetBehaviourConfig &lhs,
                           const manager_core::JointTargetBehaviourConfig &rhs)
    {
      if (lhs.joint_names != rhs.joint_names || lhs.targets.size() != rhs.targets.size())
      {
        return false;
      }

      for (std::size_t index = 0; index < lhs.targets.size(); ++index)
      {
        if (lhs.targets[index].name != rhs.targets[index].name ||
            lhs.targets[index].positions != rhs.targets[index].positions)
        {
          return false;
        }
      }

      return true;
    }

    bool managerConfigsEqual(const manager_core::ManagerConfig &lhs,
                             const manager_core::ManagerConfig &rhs)
    {
      return lhs.jaco.min_radius == rhs.jaco.min_radius &&
             lhs.jaco.max_angular_velocity == rhs.jaco.max_angular_velocity &&
             lhs.snake.gain == rhs.snake.gain &&
             jointTargetsEqual(lhs.joint_targets, rhs.joint_targets);
    }

    cartesian_manager::Params updatedParamsForRequest(
        cartesian_manager::Params params, const std::vector<rclcpp::Parameter> &parameters)
    {
      for (const auto &param : parameters)
      {
        const auto &name = param.get_name();
        if (name == "update_rate_hz")
        {
          params.update_rate_hz = param.as_double();
        }
        else if (name == "output_frame_id")
        {
          params.output_frame_id = param.as_string();
        }
        else if (name == "default_input_frame_id")
        {
          params.default_input_frame_id = param.as_string();
        }
        else if (name == "hybrid_frame_cone_angle_deg")
        {
          params.hybrid_frame_cone_angle_deg = param.as_double();
        }
        else if (name == "topics.joystick_command")
        {
          params.topics.joystick_command = param.as_string();
        }
        else if (name == "topics.visual_servoing_command")
        {
          params.topics.visual_servoing_command = param.as_string();
        }
        else if (name == "topics.mode_request")
        {
          params.topics.mode_request = param.as_string();
        }
        else if (name == "topics.ee_pose")
        {
          params.topics.ee_pose = param.as_string();
        }
        else if (name == "topics.ee_vel")
        {
          params.topics.ee_vel = param.as_string();
        }
        else if (name == "topics.ee_jac")
        {
          params.topics.ee_jac = param.as_string();
        }
        else if (name == "topics.joint_states")
        {
          params.topics.joint_states = param.as_string();
        }
        else if (name == "topics.joint_target_command")
        {
          params.topics.joint_target_command = param.as_string();
        }
        else if (name == "topics.output_command")
        {
          params.topics.output_command = param.as_string();
        }
        else if (name == "inputs.sources")
        {
          params.inputs.sources = param.as_string_array();
        }
        else if (name == "inputs.joystick.timeout_sec")
        {
          params.inputs.joystick.timeout_sec = param.as_double();
        }
        else if (name == "inputs.joystick.enabled")
        {
          params.inputs.joystick.enabled = param.as_bool();
        }
        else if (name == "inputs.visual_servoing.timeout_sec")
        {
          params.inputs.visual_servoing.timeout_sec = param.as_double();
        }
        else if (name == "inputs.visual_servoing.enabled")
        {
          params.inputs.visual_servoing.enabled = param.as_bool();
        }
        else if (name == "shapers.jaco.min_radius")
        {
          params.shapers.jaco.min_radius = param.as_double();
        }
        else if (name == "shapers.jaco.max_angular_velocity")
        {
          params.shapers.jaco.max_angular_velocity = param.as_double();
        }
        else if (name == "shapers.snake.gain")
        {
          params.shapers.snake.gain = param.as_double();
        }
        else if (name == "behaviours.joint_targets.joint_names")
        {
          params.behaviours.joint_targets.joint_names = param.as_string_array();
        }
        else if (name == "behaviours.joint_targets.target_names")
        {
          params.behaviours.joint_targets.target_names = param.as_string_array();
        }
        else if (name == "behaviours.joint_targets.positions")
        {
          params.behaviours.joint_targets.positions = param.as_double_array();
        }
      }

      return params;
    }

    double stampSec(const builtin_interfaces::msg::Time &stamp, const double fallback_sec)
    {
      if (stamp.sec == 0 && stamp.nanosec == 0)
      {
        return fallback_sec;
      }

      return rclcpp::Time(stamp).seconds();
    }

    std::string frameOrDefault(const std::string &frame_id, const std::string &default_frame_id)
    {
      return frame_id.empty() ? default_frame_id : frame_id;
    }

    manager_core::CartesianVelocity twistToCommand(const geometry_msgs::msg::TwistStamped &msg,
                                                   const std::string &default_frame_id)
    {
      manager_core::CartesianVelocity command;
      command.linear = Eigen::Vector3d(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
      command.angular =
          Eigen::Vector3d(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
      command.frame_id = frameOrDefault(msg.header.frame_id, default_frame_id);
      return command;
    }

    manager_core::CartesianVelocity twistToCommand(
        const extender_msgs::msg::CartesianVelocityCommand &msg,
        const std::string &default_frame_id, const Eigen::Quaterniond &current_orientation,
        double hybrid_frame_cone_angle_rad,
        manager_core::HybridOrientationFrame &hybrid_orientation_frame)
    {
      manager_core::CartesianVelocity command;
      command.linear = Eigen::Vector3d(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
      const Eigen::Vector3d angular_input(msg.twist.angular.x, msg.twist.angular.y,
                                          msg.twist.angular.z);
      command.angular = manager_core::mapAngularInputToBase(
          msg.orientation_frame_id, current_orientation, angular_input,
          hybrid_frame_cone_angle_rad, 0.0, hybrid_orientation_frame);
      command.frame_id = frameOrDefault(msg.header.frame_id, default_frame_id);
      return command;
    }

    bool isSupportedOrientationFrame(const std::string &orientation_frame_id)
    {
      using Command = extender_msgs::msg::CartesianVelocityCommand;
      return orientation_frame_id == Command::BASE_FRAME ||
             orientation_frame_id == Command::EFFECTOR_FRAME ||
             orientation_frame_id == Command::HYBRID_FRAME;
    }

    bool requiresEndEffectorPose(const std::string &orientation_frame_id)
    {
      using Command = extender_msgs::msg::CartesianVelocityCommand;
      return orientation_frame_id == Command::EFFECTOR_FRAME ||
             orientation_frame_id == Command::HYBRID_FRAME;
    }

    bool isValidQuaternion(const Eigen::Quaterniond &orientation)
    {
      return std::isfinite(orientation.w()) && std::isfinite(orientation.x()) &&
             std::isfinite(orientation.y()) && std::isfinite(orientation.z()) &&
             orientation.norm() > kQuaternionEpsilon;
    }

    std::optional<double> inputTimeoutSec(const ManagerConfig &config,
                                          manager_core::InputSource source)
    {
      for (const auto &input : config.inputs)
      {
        if (input.source == source)
        {
          return input.timeout_sec;
        }
      }
      return std::nullopt;
    }

    geometry_msgs::msg::TwistStamped commandToMsg(const manager_core::CartesianVelocity &command,
                                                  const rclcpp::Time &stamp,
                                                  const std::string &output_frame_id)
    {
      geometry_msgs::msg::TwistStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = frameOrDefault(command.frame_id, output_frame_id);
      msg.twist.linear.x = command.linear.x();
      msg.twist.linear.y = command.linear.y();
      msg.twist.linear.z = command.linear.z();
      msg.twist.angular.x = command.angular.x();
      msg.twist.angular.y = command.angular.y();
      msg.twist.angular.z = command.angular.z();
      return msg;
    }

    sensor_msgs::msg::JointState jointTargetToMsg(
        const manager_core::JointTargetCommand &command, const rclcpp::Time &stamp)
    {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = stamp;
      msg.name = command.joint_names;
      msg.position = command.positions;
      return msg;
    }

    std::optional<Eigen::MatrixXd> jacobianFromMsg(const std_msgs::msg::Float64MultiArray &msg,
                                                   const rclcpp::Logger &logger)
    {
      if (msg.data.empty())
      {
        return Eigen::MatrixXd{};
      }

      std::size_t rows = 0;
      std::size_t cols = 0;
      std::size_t data_offset = 0;
      std::size_t row_stride = 0;
      if (msg.layout.dim.size() == 2)
      {
        rows = msg.layout.dim[0].size;
        cols = msg.layout.dim[1].size;
        data_offset = msg.layout.data_offset;
        row_stride = msg.layout.dim[1].stride == 0 ? cols : msg.layout.dim[1].stride;
      }
      else if (msg.data.size() % 6 == 0)
      {
        rows = 6;
        cols = msg.data.size() / rows;
        row_stride = cols;
      }
      else
      {
        RCLCPP_WARN(logger, "Ignoring ee_jac message without a 2D layout and non-6xN data");
        return std::nullopt;
      }

      if (rows == 0 || cols == 0 || row_stride < cols || data_offset >= msg.data.size())
      {
        RCLCPP_WARN(logger, "Ignoring ee_jac message with inconsistent dimensions");
        return std::nullopt;
      }

      const auto last_index = data_offset + (rows - 1) * row_stride + (cols - 1);
      if (last_index >= msg.data.size())
      {
        RCLCPP_WARN(logger, "Ignoring ee_jac message with inconsistent dimensions");
        return std::nullopt;
      }

      Eigen::MatrixXd jacobian(rows, cols);
      for (std::size_t row = 0; row < rows; ++row)
      {
        for (std::size_t col = 0; col < cols; ++col)
        {
          jacobian(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
              msg.data[data_offset + row * row_stride + col];
        }
      }

      return jacobian;
    }
  } // namespace

  CartesianManagerROS::CartesianManagerROS(const rclcpp::NodeOptions &options)
      : rclcpp::Node("cartesian_manager", options), topic_manager_(*this)
  {
    readParameters();
    applyConfig(config_, true);
    recreateTimer();
  }

  void CartesianManagerROS::readParameters()
  {
    param_listener_ = std::make_shared<cartesian_manager::ParamListener>(this);
    params_ = param_listener_->get_params();
    config_ = parseManagerConfig(params_);
    parameter_validator_handle_ =
        add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) {
          return validateParameterUpdate(parameters);
        });
  }

  void CartesianManagerROS::applyConfig(const ManagerConfig &config, bool force_rebuild)
  {
    const auto previous_config = config_;
    const bool manager_config_changed =
        force_rebuild || !managerConfigsEqual(previous_config.manager, config.manager);
    const bool input_frame_changed =
        force_rebuild ||
        previous_config.frames.default_input_frame_id != config.frames.default_input_frame_id;
    const bool input_sources_changed =
        force_rebuild || !inputSourcesEqual(previous_config.inputs, config.inputs);
    const bool input_config_changed = force_rebuild || input_frame_changed ||
                                      input_sources_changed ||
                                      !inputConfigsEqual(previous_config.inputs, config.inputs);
    const bool ros_interfaces_changed = force_rebuild || input_sources_changed ||
                                        !usedTopicsEqual(previous_config.topics, config.topics);
    const bool timer_rate_changed =
        force_rebuild || previous_config.update_rate_hz != config.update_rate_hz;
    const bool hybrid_cone_changed =
        force_rebuild || previous_config.hybrid_frame_cone_angle_deg !=
                             config.hybrid_frame_cone_angle_deg;

    config_ = config;

    if (manager_config_changed)
    {
      manager_.configure(config_.manager);
    }

    if (input_config_changed)
    {
      manager_.setInputFrameId(config_.frames.default_input_frame_id);
      if (input_frame_changed || input_sources_changed)
      {
        manager_.clearInputChannels();
        hybrid_orientation_frame_.reset();
        last_joystick_receipt_sec_.reset();
        last_orientation_frame_id_.clear();
        if (input_frame_changed)
        {
          ee_pose_received_ = false;
        }
      }
      for (const auto &input : config_.inputs)
      {
        manager_.addInputChannel(input.source, input.timeout_sec, input.enabled);
      }
    }

    if (hybrid_cone_changed)
    {
      hybrid_orientation_frame_.reset();
    }

    if (ros_interfaces_changed)
    {
      clearRosInterfaces();
      setupPublishers();
      setupSubscribers();
    }

    if (timer_rate_changed && timer_)
    {
      recreateTimer();
    }
  }

  void CartesianManagerROS::clearRosInterfaces()
  {
    topic_manager_.removePublisher(kOutputCommandPublisher);
    topic_manager_.removePublisher(kJointTargetCommandPublisher);

    topic_manager_.removeSubscriber("mode_request");
    topic_manager_.removeSubscriber("ee_pose");
    topic_manager_.removeSubscriber("ee_vel");
    topic_manager_.removeSubscriber("ee_jac");
    topic_manager_.removeSubscriber("joint_states");
    topic_manager_.removeSubscriber("joystick_command");
    topic_manager_.removeSubscriber("visual_servoing_command");
  }

  void CartesianManagerROS::recreateTimer()
  {
    if (timer_)
    {
      timer_->cancel();
    }

    timer_ = create_wall_timer(timerPeriod(config_.update_rate_hz), [this]() { updateVelocity(); });
  }

  void CartesianManagerROS::refreshParameters()
  {
    auto updated_params = params_;
    if (!param_listener_ || !param_listener_->try_update_params(updated_params))
    {
      return;
    }

    ManagerConfig updated_config;
    try
    {
      updated_config = parseManagerConfig(updated_params);
    }
    catch (const std::invalid_argument &error)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Ignoring invalid runtime parameter update: %s", error.what());
      return;
    }

    params_ = updated_params;
    applyConfig(updated_config, false);

    RCLCPP_INFO(get_logger(), "Applied updated cartesian_manager parameters");
  }

  rcl_interfaces::msg::SetParametersResult CartesianManagerROS::validateParameterUpdate(
      const std::vector<rclcpp::Parameter> &parameters) const
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    try
    {
      parseManagerConfig(updatedParamsForRequest(params_, parameters));
    }
    catch (const std::exception &error)
    {
      result.successful = false;
      result.reason = error.what();
    }

    return result;
  }

  void CartesianManagerROS::setupPublishers()
  {
    topic_manager_.addPublisher<geometry_msgs::msg::TwistStamped>(kOutputCommandPublisher,
                                                                  config_.topics.output_command);
    topic_manager_.addPublisher<sensor_msgs::msg::JointState>(kJointTargetCommandPublisher,
                                                              config_.topics.joint_target_command);
  }

  void CartesianManagerROS::setupSubscribers()
  {
    topic_manager_.addSubscriber<std_msgs::msg::String>(
        "mode_request", config_.topics.mode_request,
        [this](const std_msgs::msg::String &msg) { modeRequestCallback(msg.data); });

    topic_manager_.addSubscriber<geometry_msgs::msg::PoseStamped>(
        "ee_pose", config_.topics.ee_pose, [this](const geometry_msgs::msg::PoseStamped &msg) {
          robot_context_.ee_pose.position =
              Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
          robot_context_.ee_pose.orientation =
              Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x,
                                 msg.pose.orientation.y, msg.pose.orientation.z);
          robot_context_.ee_pose.frame_id =
              frameOrDefault(msg.header.frame_id, config_.frames.default_input_frame_id);
          const bool pose_is_valid = isValidQuaternion(robot_context_.ee_pose.orientation) &&
                                     robot_context_.ee_pose.frame_id ==
                                         config_.frames.default_input_frame_id;
          if (!pose_is_valid)
          {
            if (ee_pose_received_)
            {
              hybrid_orientation_frame_.reset();
            }
            ee_pose_received_ = false;
            if (requiresEndEffectorPose(last_orientation_frame_id_))
            {
              manager_.clearInputCommand(manager_core::InputSource::JOYSTICK);
              last_joystick_receipt_sec_.reset();
              last_orientation_frame_id_.clear();
            }
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Ignoring end-effector pose with an invalid orientation or frame '%s'",
                robot_context_.ee_pose.frame_id.c_str());
          }
          else
          {
            ee_pose_received_ = true;
          }
        });

    topic_manager_.addSubscriber<geometry_msgs::msg::TwistStamped>(
        "ee_vel", config_.topics.ee_vel, [this](const geometry_msgs::msg::TwistStamped &msg) {
          robot_context_.ee_vel = twistToCommand(msg, config_.frames.default_input_frame_id);
        });

    topic_manager_.addSubscriber<std_msgs::msg::Float64MultiArray>(
        "ee_jac", config_.topics.ee_jac, [this](const std_msgs::msg::Float64MultiArray &msg) {
          auto jacobian = jacobianFromMsg(msg, get_logger());
          if (jacobian)
          {
            robot_context_.ee_jac = std::move(*jacobian);
          }
        });

    topic_manager_.addSubscriber<sensor_msgs::msg::JointState>(
        "joint_states", config_.topics.joint_states,
        [this](const sensor_msgs::msg::JointState &msg) {
          robot_context_.joint_names = msg.name;
          if (msg.position.empty())
          {
            robot_context_.joint_positions = Eigen::VectorXd{};
            return;
          }

          robot_context_.joint_positions = Eigen::Map<const Eigen::VectorXd>(
              msg.position.data(), static_cast<Eigen::Index>(msg.position.size()));
        });

    if (hasInputSource(config_, manager_core::InputSource::JOYSTICK))
    {
      topic_manager_.addSubscriber<extender_msgs::msg::CartesianVelocityCommand>(
          "joystick_command", config_.topics.joystick_command,
          [this](const extender_msgs::msg::CartesianVelocityCommand &msg) {
            const auto now_sec = topic_manager_.nowSec();
            const auto input_frame_id =
                frameOrDefault(msg.header.frame_id, config_.frames.default_input_frame_id);
            if (input_frame_id != config_.frames.default_input_frame_id)
            {
              RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 5000,
                  "Ignoring joystick command in frame '%s'; expected input frame '%s'",
                  input_frame_id.c_str(), config_.frames.default_input_frame_id.c_str());
              return;
            }

            if (!isSupportedOrientationFrame(msg.orientation_frame_id))
            {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                   "Ignoring joystick command with invalid orientation frame '%s'",
                                   msg.orientation_frame_id.c_str());
              return;
            }

            if (requiresEndEffectorPose(msg.orientation_frame_id) && !ee_pose_received_)
            {
              RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 5000,
                  "Ignoring %s joystick command until a valid end-effector pose is received",
                  msg.orientation_frame_id.c_str());
              return;
            }

            const auto joystick_timeout =
                inputTimeoutSec(config_, manager_core::InputSource::JOYSTICK);
            if (last_joystick_receipt_sec_ && joystick_timeout &&
                now_sec - *last_joystick_receipt_sec_ > *joystick_timeout)
            {
              hybrid_orientation_frame_.reset();
            }

            using Command = extender_msgs::msg::CartesianVelocityCommand;
            if (msg.orientation_frame_id != last_orientation_frame_id_ &&
                (msg.orientation_frame_id == Command::HYBRID_FRAME ||
                 last_orientation_frame_id_ == Command::HYBRID_FRAME))
            {
              hybrid_orientation_frame_.reset();
            }

            const double hybrid_cone_rad =
                config_.hybrid_frame_cone_angle_deg * kPi / 180.0;
            const auto command =
                twistToCommand(msg, config_.frames.default_input_frame_id,
                               robot_context_.ee_pose.orientation, hybrid_cone_rad,
                               hybrid_orientation_frame_);
            if (!manager_.setInputCommand(manager_core::InputSource::JOYSTICK, command,
                                          stampSec(msg.header.stamp, now_sec)))
            {
              RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 5000,
                  "Ignoring joystick command in frame '%s'; expected input frame '%s'",
                  command.frame_id.c_str(), config_.frames.default_input_frame_id.c_str());
              return;
            }
            last_joystick_receipt_sec_ = now_sec;
            last_orientation_frame_id_ = msg.orientation_frame_id;
          });
    }

    if (hasInputSource(config_, manager_core::InputSource::VISUAL_SERVOING))
    {
      topic_manager_.addSubscriber<geometry_msgs::msg::TwistStamped>(
          "visual_servoing_command", config_.topics.visual_servoing_command,
          [this](const geometry_msgs::msg::TwistStamped &msg) {
            const auto now_sec = topic_manager_.nowSec();
            const auto command = twistToCommand(msg, config_.frames.default_input_frame_id);
            if (!manager_.setInputCommand(manager_core::InputSource::VISUAL_SERVOING, command,
                                          stampSec(msg.header.stamp, now_sec)))
            {
              RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 5000,
                  "Ignoring visual-servoing command in frame '%s'; expected input frame '%s'",
                  command.frame_id.c_str(), config_.frames.default_input_frame_id.c_str());
            }
          });
    }
  }

  void CartesianManagerROS::modeRequestCallback(const std::string &mode_request)
  {
    const auto normalized_mode_request = normalizeParameterName(mode_request);
    const bool joint_target_request =
        normalized_mode_request.rfind(kJointTargetModePrefix, 0) == 0;
    const bool passthrough_request = normalized_mode_request == kBehaviourPassthroughMode;

    if (!manager_.setMode(normalized_mode_request))
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid mode request '%s'", mode_request.c_str());
      return;
    }

    if (joint_target_request)
    {
      publishJointTargetCommand(manager_.activeJointTargetCommand());
      manager_.setMode(kBehaviourPassthroughMode);
      return;
    }

    if (passthrough_request)
    {
      publishJointTargetCommand(std::nullopt);
    }
  }

  void CartesianManagerROS::publishJointTargetCommand(
      const std::optional<manager_core::JointTargetCommand> &command)
  {
    if (command)
    {
      topic_manager_.publish(kJointTargetCommandPublisher, jointTargetToMsg(*command, now()));
      return;
    }

    sensor_msgs::msg::JointState cancel_msg;
    cancel_msg.header.stamp = now();
    topic_manager_.publish(kJointTargetCommandPublisher, cancel_msg);
  }

  void CartesianManagerROS::updateVelocity()
  {
    refreshParameters();

    const auto now = this->now();
    const auto command =
        manager_.update(now.seconds(), 1.0 / config_.update_rate_hz, robot_context_)
            .value_or(manager_core::CartesianVelocity{});
    topic_manager_.publish(kOutputCommandPublisher,
                           commandToMsg(command, now, config_.frames.output_frame_id));
  }
} // namespace ros_cartesian_manager
