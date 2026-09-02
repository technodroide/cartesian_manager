#include "cartesian_manager/ros/parameter_parsing.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ros_cartesian_manager
{
  namespace
  {
    constexpr double kMaxUpdateRateHz = 1000.0;

    bool isFinite(double value)
    {
      return std::isfinite(value);
    }

    void requireNonEmpty(const std::string &value, const std::string &name)
    {
      if (value.empty())
      {
        throw std::invalid_argument(name + " must not be empty");
      }
    }

    void requirePositive(double value, const std::string &name)
    {
      if (!isFinite(value) || value <= 0.0)
      {
        throw std::invalid_argument(name + " must be finite and > 0.0");
      }
    }

    void requireAtMost(double value, double max_value, const std::string &name)
    {
      if (!isFinite(value) || value > max_value)
      {
        throw std::invalid_argument(name + " must be finite and <= " + std::to_string(max_value));
      }
    }

    void requireNonNegative(double value, const std::string &name)
    {
      if (!isFinite(value) || value < 0.0)
      {
        throw std::invalid_argument(name + " must be finite and >= 0.0");
      }
    }

    std::vector<std::string> normalizedNonEmptyNames(const std::vector<std::string> &names)
    {
      std::vector<std::string> normalized_names;
      normalized_names.reserve(names.size());

      for (const auto &name : names)
      {
        auto normalized = normalizeParameterName(name);
        if (!normalized.empty())
        {
          normalized_names.push_back(std::move(normalized));
        }
      }

      return normalized_names;
    }

    void requireUniqueNames(const std::vector<std::string> &names, const std::string &name)
    {
      std::unordered_set<std::string> seen;
      for (const auto &value : names)
      {
        if (!seen.insert(value).second)
        {
          throw std::invalid_argument(name + " contains duplicate entry '" + value + "'");
        }
      }
    }

    manager_core::InputSource inputSourceFromName(const std::string &name)
    {
      if (name == "joystick")
      {
        return manager_core::InputSource::JOYSTICK;
      }

      if (name == "visual_servoing")
      {
        return manager_core::InputSource::VISUAL_SERVOING;
      }

      throw std::invalid_argument("inputs.sources contains unsupported source '" + name + "'");
    }

    InputConfig makeInputConfig(manager_core::InputSource source,
                                const cartesian_manager::Params &params)
    {
      switch (source)
      {
      case manager_core::InputSource::JOYSTICK:
        requirePositive(params.inputs.joystick.timeout_sec, "inputs.joystick.timeout_sec");
        return InputConfig{source, params.inputs.joystick.timeout_sec,
                           params.inputs.joystick.enabled};

      case manager_core::InputSource::VISUAL_SERVOING:
        requirePositive(params.inputs.visual_servoing.timeout_sec,
                        "inputs.visual_servoing.timeout_sec");
        return InputConfig{source, params.inputs.visual_servoing.timeout_sec,
                           params.inputs.visual_servoing.enabled};
      }

      throw std::invalid_argument("inputs.sources contains an unknown source enum value");
    }

    std::vector<manager_core::JointTarget> makeJointTargets(
        const std::vector<std::string> &target_names, const std::vector<double> &positions,
        std::size_t joint_count)
    {
      std::vector<manager_core::JointTarget> targets;
      if (joint_count == 0 || target_names.empty())
      {
        return targets;
      }

      const auto expected_position_count = target_names.size() * joint_count;
      if (positions.size() != expected_position_count)
      {
        throw std::invalid_argument("behaviours.joint_targets.positions has " +
                                    std::to_string(positions.size()) + " values, expected " +
                                    std::to_string(expected_position_count));
      }

      targets.reserve(target_names.size());
      for (std::size_t target_index = 0; target_index < target_names.size(); ++target_index)
      {
        const auto first_position = target_index * joint_count;
        const auto first = positions.begin() + static_cast<std::ptrdiff_t>(first_position);
        const auto last = first + static_cast<std::ptrdiff_t>(joint_count);
        if (!std::all_of(first, last, isFinite))
        {
          throw std::invalid_argument(
              "behaviours.joint_targets.positions contains a non-finite value");
        }

        manager_core::JointTarget target;
        target.name = target_names[target_index];
        target.positions.assign(first, last);
        targets.push_back(std::move(target));
      }

      return targets;
    }

    manager_core::JointTargetBehaviourConfig makeJointTargetConfig(
        const cartesian_manager::Params &params)
    {
      const auto &joint_target_params = params.behaviours.joint_targets;

      manager_core::JointTargetBehaviourConfig config;
      config.joint_names = normalizedNonEmptyNames(joint_target_params.joint_names);
      requireUniqueNames(config.joint_names, "behaviours.joint_targets.joint_names");

      auto target_names = normalizedNonEmptyNames(joint_target_params.target_names);
      requireUniqueNames(target_names, "behaviours.joint_targets.target_names");

      config.targets =
          makeJointTargets(target_names, joint_target_params.positions, config.joint_names.size());
      return config;
    }
  } // namespace

  std::string normalizeParameterName(std::string name)
  {
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::replace(name.begin(), name.end(), '-', '_');
    return name;
  }

  bool hasInputSource(const ManagerConfig &config, manager_core::InputSource source)
  {
    return std::any_of(config.inputs.begin(), config.inputs.end(),
                       [source](const InputConfig &input) { return input.source == source; });
  }

  ManagerConfig parseManagerConfig(const cartesian_manager::Params &params)
  {
    ManagerConfig config;

    config.update_rate_hz = params.update_rate_hz;
    requirePositive(config.update_rate_hz, "update_rate_hz");
    requireAtMost(config.update_rate_hz, kMaxUpdateRateHz, "update_rate_hz");

    config.hybrid_frame_cone_angle_deg = params.hybrid_frame_cone_angle_deg;
    requirePositive(config.hybrid_frame_cone_angle_deg, "hybrid_frame_cone_angle_deg");
    if (config.hybrid_frame_cone_angle_deg >= 90.0)
    {
      throw std::invalid_argument("hybrid_frame_cone_angle_deg must be < 90.0");
    }

    config.topics.joystick_command = params.topics.joystick_command;
    config.topics.visual_servoing_command = params.topics.visual_servoing_command;
    config.topics.mode_request = params.topics.mode_request;
    config.topics.ee_pose = params.topics.ee_pose;
    config.topics.ee_vel = params.topics.ee_vel;
    config.topics.ee_jac = params.topics.ee_jac;
    config.topics.joint_states = params.topics.joint_states;
    config.topics.joint_target_command = params.topics.joint_target_command;
    config.topics.output_command = params.topics.output_command;

    requireNonEmpty(config.topics.joystick_command, "topics.joystick_command");
    requireNonEmpty(config.topics.visual_servoing_command, "topics.visual_servoing_command");
    requireNonEmpty(config.topics.mode_request, "topics.mode_request");
    requireNonEmpty(config.topics.ee_pose, "topics.ee_pose");
    requireNonEmpty(config.topics.ee_vel, "topics.ee_vel");
    requireNonEmpty(config.topics.ee_jac, "topics.ee_jac");
    requireNonEmpty(config.topics.joint_states, "topics.joint_states");
    requireNonEmpty(config.topics.joint_target_command, "topics.joint_target_command");
    requireNonEmpty(config.topics.output_command, "topics.output_command");

    config.frames.output_frame_id = params.output_frame_id;
    config.frames.default_input_frame_id = params.default_input_frame_id;
    requireNonEmpty(config.frames.output_frame_id, "output_frame_id");
    requireNonEmpty(config.frames.default_input_frame_id, "default_input_frame_id");

    const auto input_names = normalizedNonEmptyNames(params.inputs.sources);
    if (input_names.size() != params.inputs.sources.size())
    {
      throw std::invalid_argument("inputs.sources must not contain empty source names");
    }
    requireUniqueNames(input_names, "inputs.sources");

    config.inputs.reserve(input_names.size());
    for (const auto &input_name : input_names)
    {
      config.inputs.push_back(makeInputConfig(inputSourceFromName(input_name), params));
    }

    config.manager.jaco.min_radius = params.shapers.jaco.min_radius;
    config.manager.jaco.max_angular_velocity = params.shapers.jaco.max_angular_velocity;
    config.manager.snake.gain = params.shapers.snake.gain;
    config.manager.joint_targets = makeJointTargetConfig(params);

    requireNonNegative(config.manager.jaco.min_radius, "shapers.jaco.min_radius");
    requireNonNegative(config.manager.jaco.max_angular_velocity,
                       "shapers.jaco.max_angular_velocity");
    requirePositive(config.manager.snake.gain, "shapers.snake.gain");

    return config;
  }
} // namespace ros_cartesian_manager
