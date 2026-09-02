#include "cartesian_manager/core/manager.hpp"

#include <memory>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

#include "cartesian_manager/core/shapers/geometric/jaco.hpp"
#include "cartesian_manager/core/shapers/geometric/snake.hpp"

namespace manager_core
{
  namespace
  {
    std::vector<std::string> splitModeRequest(const std::string &mode_request)
    {
      std::vector<std::string> parts;
      std::stringstream stream(mode_request);
      std::string part;

      while (std::getline(stream, part, '/'))
      {
        if (part.empty())
        {
          return {};
        }
        parts.push_back(part);
      }

      return parts;
    }
  } // namespace

  void Manager::configure(const ManagerConfig &config)
  {
    geometric_shapers_.clear();
    behaviours_.clear();

    registerGeometricShaper(Geometrics::JACO, std::make_unique<JacoShaper>(config.jaco));
    registerGeometricShaper(Geometrics::SNAKE, std::make_unique<SnakeShaper>(config.snake));
    joint_target_config_ = config.joint_targets;

    if (behaviour_state_ == Behaviours::JOINT_TARGET && !jointTargetByName(joint_target_name_))
    {
      behaviour_state_ = Behaviours::PASSTHROUGH;
    }
  }

  void Manager::setInputFrameId(const std::string &frame_id)
  {
    input_manager_.setFrameId(frame_id);
  }

  void Manager::addInputChannel(InputSource source, double timeout_sec, bool enabled)
  {
    input_manager_.addInputChannel(source, timeout_sec, enabled);
  }

  void Manager::clearInputChannels()
  {
    input_manager_.clearInputChannels();
  }

  void Manager::enableInput(InputSource source)
  {
    input_manager_.enableInputChannel(source);
  }

  void Manager::disableInput(InputSource source)
  {
    input_manager_.disableInputChannel(source);
  }

  bool Manager::setInputCommand(InputSource source, const CartesianVelocity &command,
                                double stamp_sec)
  {
    return input_manager_.setCommand(source, command, stamp_sec);
  }

  void Manager::clearInputCommand(InputSource source)
  {
    input_manager_.clearCommand(source);
  }

  std::vector<InputSource> Manager::getValidInputSources(double now_sec) const
  {
    return input_manager_.getValidSources(now_sec);
  }

  bool Manager::setMode(const std::string &mode_request)
  {
    const auto parts = splitModeRequest(mode_request);
    if (parts.size() < 2)
    {
      return false;
    }

    if (parts[0] == "geometric")
    {
      if (parts.size() != 2)
      {
        return false;
      }

      if (parts[1] == "both")
      {
        geometric_state_ = Geometrics::BOTH;
      }
      else if (parts[1] == "jaco")
      {
        geometric_state_ = Geometrics::JACO;
      }
      else if (parts[1] == "snake")
      {
        geometric_state_ = Geometrics::SNAKE;
      }
      else
      {
        return false;
      }

      return true;
    }

    if (parts[0] == "behaviour")
    {
      if (parts[1] == "passthrough")
      {
        if (parts.size() != 2)
        {
          return false;
        }

        behaviour_state_ = Behaviours::PASSTHROUGH;
        return true;
      }

      if (parts[1] == "joint_target")
      {
        if (parts.size() != 3)
        {
          return false;
        }

        if (!jointTargetByName(parts[2]))
        {
          return false;
        }

        joint_target_name_ = parts[2];
        behaviour_state_ = Behaviours::JOINT_TARGET;
        return true;
      }
    }

    return false;
  }

  std::optional<JointTargetCommand> Manager::activeJointTargetCommand() const
  {
    if (behaviour_state_ != Behaviours::JOINT_TARGET)
    {
      return std::nullopt;
    }

    const auto target = jointTargetByName(joint_target_name_);
    if (!target)
    {
      return std::nullopt;
    }

    JointTargetCommand command;
    command.name = target->name;
    command.joint_names = joint_target_config_.joint_names;
    command.positions = target->positions;
    return command;
  }

  void Manager::applyGeometric(CartesianVelocity &command, const RobotContext &context,
                               double dt_sec)
  {
    if (geometric_state_ == Geometrics::BOTH)
      return;

    const auto shaper = geometric_shapers_.find(geometric_state_);
    if (shaper == geometric_shapers_.end())
    {
      return;
    }

    command = shaper->second->update(command, context, dt_sec);
  }

  void Manager::applyBehaviour(CartesianVelocity &command, const RobotContext &context,
                               double dt_sec)
  {
    if (behaviour_state_ == Behaviours::PASSTHROUGH)
      return;

    const auto behaviour = behaviours_.find(behaviour_state_);
    if (behaviour == behaviours_.end())
    {
      return;
    }

    command = behaviour->second->update(command, context, dt_sec);
  }

  void Manager::registerGeometricShaper(Geometrics state, std::unique_ptr<Shaper> shaper)
  {
    geometric_shapers_[state] = std::move(shaper);
  }

  void Manager::registerBehaviour(Behaviours state, std::unique_ptr<Shaper> shaper)
  {
    behaviours_[state] = std::move(shaper);
  }

  const JointTarget *Manager::jointTargetByName(const std::string &target_name) const
  {
    for (const auto &target : joint_target_config_.targets)
    {
      if (target.name == target_name)
      {
        return &target;
      }
    }

    return nullptr;
  }

  std::optional<CartesianCommand> Manager::update(double now_sec, double dt_sec,
                                                  const RobotContext &context)
  {
    if (behaviour_state_ == Behaviours::JOINT_TARGET)
    {
      return CartesianVelocity{};
    }

    auto command = input_manager_.getFullCommand(now_sec);
    if (command)
    {
      applyGeometric(*command, context, dt_sec);
      applyBehaviour(*command, context, dt_sec);
    }
    return command;
  }
} // namespace manager_core
