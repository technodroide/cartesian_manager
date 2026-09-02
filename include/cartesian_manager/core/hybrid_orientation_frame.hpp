#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace manager_core
{
  class HybridOrientationFrame
  {
  public:
    void reset();

    Eigen::Matrix3d update(const Eigen::Quaterniond &current_orientation,
                           const Eigen::Vector3d &angular_input, double cone_angle_rad,
                           double release_threshold);

  private:
    bool initialized_{false};
    bool has_last_active_input_{false};
    Eigen::Vector3d x_axis_{Eigen::Vector3d::UnitX()};
    Eigen::Vector3d last_active_input_{Eigen::Vector3d::Zero()};
  };

  Eigen::Vector3d mapAngularInputToBase(const std::string &orientation_frame_id,
                                       const Eigen::Quaterniond &current_orientation,
                                       const Eigen::Vector3d &angular_input,
                                       double cone_angle_rad, double release_threshold,
                                       HybridOrientationFrame &hybrid_frame,
                                       Eigen::Matrix3d *orientation_frame_to_base = nullptr);
} // namespace manager_core
