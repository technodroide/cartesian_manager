#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "cartesian_manager/core/input_manager.hpp"

#include "cartesian_manager/core/shapers/behaviour/joint_target.hpp"
#include "cartesian_manager/core/shapers/geometric/jaco.hpp"
#include "cartesian_manager/core/shapers/geometric/snake.hpp"
#include "cartesian_manager/core/shapers/shaper.hpp"

#include "cartesian_manager/core/types.hpp"

namespace manager_core
{
  struct ManagerConfig
  {
    JacoShaperConfig jaco;
    SnakeShaperConfig snake;
    JointTargetBehaviourConfig joint_targets;
  };

  class Manager
  {
  public:
    void configure(const ManagerConfig &config);
    void setInputFrameId(const std::string &frame_id);

    void addInputChannel(InputSource source, double timeout_sec, bool enabled = true);
    void clearInputChannels();

    void enableInput(InputSource source);
    void disableInput(InputSource source);

    bool setInputCommand(InputSource source, const CartesianVelocity &command, double stamp_sec);
    void clearInputCommand(InputSource source);
    std::vector<InputSource> getValidInputSources(double now_sec) const;

    bool setMode(const std::string &mode_request);
    std::optional<JointTargetCommand> activeJointTargetCommand() const;

    std::optional<CartesianVelocity> update(double now_sec, double dt_sec,
                                            const RobotContext &context);

  private:
    void applyGeometric(CartesianVelocity &command, const RobotContext &context, double dt_sec);
    void applyBehaviour(CartesianVelocity &command, const RobotContext &context, double dt_sec);

    void registerGeometricShaper(Geometrics state, std::unique_ptr<Shaper> shaper);
    void registerBehaviour(Behaviours state, std::unique_ptr<Shaper> shaper);
    const JointTarget *jointTargetByName(const std::string &target_name) const;

    InputManager input_manager_;
    Geometrics geometric_state_{Geometrics::BOTH};
    Behaviours behaviour_state_{Behaviours::PASSTHROUGH};
    JointTargetBehaviourConfig joint_target_config_;

    std::unordered_map<Geometrics, std::unique_ptr<Shaper>> geometric_shapers_;
    std::unordered_map<Behaviours, std::unique_ptr<Shaper>> behaviours_;
    std::string joint_target_name_{"home"};
  };
} // namespace manager_core
