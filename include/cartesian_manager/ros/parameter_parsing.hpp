#pragma once

#include <string>
#include <vector>

#include "cartesian_manager/cartesian_manager_parameters.hpp"
#include "cartesian_manager/core/manager.hpp"
#include "cartesian_manager/core/types.hpp"

namespace ros_cartesian_manager
{
  /**
   * @brief ROS topic names loaded from cartesian_manager.yaml.
   *
   * These strings are used by CartesianManagerROS when creating TopicManager publishers and
   * subscribers. They are kept separate from the core manager configuration because the core
   * manager has no dependency on ROS topics.
   */
  struct TopicConfig
  {
    std::string joystick_command;
    std::string visual_servoing_command;
    std::string mode_request;
    std::string output_command;
    std::string ee_pose;
    std::string ee_vel;
    std::string ee_jac;
    std::string joint_states;
    std::string joint_target_command;
  };

  /**
   * @brief Frame names used by the ROS bridge.
   *
   * output_frame_id is the frame used for published Cartesian commands. default_input_frame_id is
   * applied when an incoming command message has an empty header frame.
   */
  struct FrameConfig
  {
    std::string output_frame_id;
    std::string default_input_frame_id;
  };

  /**
   * @brief Runtime input-channel configuration for manager_core::Manager.
   *
   * Each entry corresponds to one source declared in inputs.sources. The enabled flag is only the
   * startup enabled state; declaration is represented by presence in ManagerConfig::inputs.
   */
  struct InputConfig
  {
    manager_core::InputSource source{manager_core::InputSource::JOYSTICK};
    double timeout_sec{0.2};
    bool enabled{true};
  };

  /**
   * @brief Complete parsed configuration for CartesianManagerROS.
   *
   * This is the bridge type between generated parameters and runtime objects. ROS-specific settings
   * stay at this level, while shaper/behaviour settings are grouped into manager for direct use
   * with manager_core::Manager::configure().
   */
  struct ManagerConfig
  {
    double update_rate_hz{100.0};
    double hybrid_frame_cone_angle_deg{5.0};
    TopicConfig topics;
    FrameConfig frames;
    std::vector<InputConfig> inputs;
    manager_core::ManagerConfig manager;
  };

  /**
   * @brief Normalize user-provided parameter names for mode/target matching.
   *
   * Converts letters to lowercase and replaces '-' with '_'. Empty strings remain empty.
   *
   * @param name Raw name from parameters or requests.
   * @return Normalized name.
   */
  std::string normalizeParameterName(std::string name);

  /**
   * @brief Check whether an input source is declared in a parsed configuration.
   *
   * This is useful in CartesianManagerROS for deciding whether to create optional subscribers, such
   * as the joystick command subscriber.
   *
   * @param config Parsed manager configuration.
   * @param source Input source to look for.
   * @return true when the source appears in config.inputs.
   */
  bool hasInputSource(const ManagerConfig &config, manager_core::InputSource source);

  /**
   * @brief Convert generated parameter-library values into runtime configuration.
   *
   * The parser validates required strings, finite/positive numeric values, joint target dimensions,
   * and duplicate joint/target names. Invalid parameters throw std::invalid_argument with a message
   * naming the offending parameter.
   *
   * @param params Generated parameter-library struct from cartesian_manager.yaml.
   * @return Runtime configuration consumed by CartesianManagerROS and manager_core::Manager.
   * @throws std::invalid_argument when a parameter value is inconsistent or invalid.
   */
  ManagerConfig parseManagerConfig(const cartesian_manager::Params &params);
} // namespace ros_cartesian_manager
