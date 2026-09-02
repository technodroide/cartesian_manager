#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

#include "cartesian_manager/cartesian_manager_parameters.hpp"
#include "cartesian_manager/core/hybrid_orientation_frame.hpp"
#include "cartesian_manager/core/manager.hpp"
#include "cartesian_manager/ros/parameter_parsing.hpp"
#include "cartesian_manager/ros/topic_manager.hpp"

namespace ros_cartesian_manager
{

  class CartesianManagerROS : public rclcpp::Node
  {
  public:
    explicit CartesianManagerROS(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    void setupSubscribers();
    void setupPublishers();
    void readParameters();
    void applyConfig(const ManagerConfig &config, bool force_rebuild);
    void clearRosInterfaces();
    void recreateTimer();
    void refreshParameters();
    void updateVelocity();

    rcl_interfaces::msg::SetParametersResult validateParameterUpdate(
        const std::vector<rclcpp::Parameter> &parameters) const;
    void modeRequestCallback(const std::string &mode_request);
    void publishJointTargetCommand(
        const std::optional<manager_core::JointTargetCommand> &command);

    TopicManager topic_manager_;
    manager_core::Manager manager_;
    manager_core::RobotContext robot_context_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<cartesian_manager::ParamListener> param_listener_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_validator_handle_;
    cartesian_manager::Params params_;
    ManagerConfig config_;
    manager_core::HybridOrientationFrame hybrid_orientation_frame_;
    bool ee_pose_received_{false};
    std::optional<double> last_joystick_receipt_sec_;
    std::string last_orientation_frame_id_;
  };
} // namespace ros_cartesian_manager
