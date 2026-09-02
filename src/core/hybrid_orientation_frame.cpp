#include "cartesian_manager/core/hybrid_orientation_frame.hpp"

#include <algorithm>
#include <cmath>

namespace manager_core
{
  namespace
  {
    constexpr double kVectorEpsilon = 1e-12;

    Eigen::Vector3d projectOntoNormalPlane(const Eigen::Vector3d &vector,
                                           const Eigen::Vector3d &plane_normal)
    {
      return vector - vector.dot(plane_normal) * plane_normal;
    }

    Eigen::Vector3d normalizedProjection(const Eigen::Vector3d &preferred_axis,
                                         const Eigen::Vector3d &plane_normal)
    {
      Eigen::Vector3d projected = projectOntoNormalPlane(preferred_axis, plane_normal);
      if (projected.norm() < kVectorEpsilon)
      {
        projected = projectOntoNormalPlane(Eigen::Vector3d::UnitY(), plane_normal);
      }
      if (projected.norm() < kVectorEpsilon)
      {
        projected = plane_normal.unitOrthogonal();
      }
      return projected.normalized();
    }
  } // namespace

  void HybridOrientationFrame::reset()
  {
    initialized_ = false;
    has_last_active_input_ = false;
    x_axis_ = Eigen::Vector3d::UnitX();
    last_active_input_.setZero();
  }

  Eigen::Matrix3d HybridOrientationFrame::update(const Eigen::Quaterniond &current_orientation,
                                                 const Eigen::Vector3d &angular_input,
                                                 double cone_angle_rad,
                                                 double release_threshold)
  {
    Eigen::Quaterniond normalized_orientation = current_orientation;
    if (normalized_orientation.norm() < kVectorEpsilon)
    {
      normalized_orientation = Eigen::Quaterniond::Identity();
    }
    else
    {
      normalized_orientation.normalize();
    }

    const Eigen::Vector3d base_x = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d base_z = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d tool_z =
        (normalized_orientation.toRotationMatrix() * Eigen::Vector3d::UnitZ()).normalized();

    const double vertical_alignment = std::clamp(base_z.dot(tool_z), -1.0, 1.0);
    const bool inside_cone = std::abs(vertical_alignment) > std::cos(cone_angle_rad);
    const bool active = angular_input.norm() > release_threshold;
    const bool reversed =
        active && has_last_active_input_ && angular_input.dot(last_active_input_) < 0.0;
    const bool reanchor = !initialized_ || !active || reversed;

    Eigen::Vector3d next_x;
    if (inside_cone)
    {
      next_x = normalizedProjection(reanchor ? base_x : x_axis_, tool_z);
    }
    else
    {
      next_x = base_z.cross(tool_z).normalized();
      if (!reanchor && next_x.dot(x_axis_) < 0.0)
      {
        next_x = -next_x;
      }
    }

    Eigen::Vector3d next_y = tool_z.cross(next_x);
    if (next_y.norm() < kVectorEpsilon)
    {
      next_x = normalizedProjection(base_x, tool_z);
      next_y = tool_z.cross(next_x);
    }
    next_y.normalize();
    next_x = next_y.cross(tool_z).normalized();

    x_axis_ = next_x;
    initialized_ = true;
    if (active)
    {
      last_active_input_ = angular_input;
      has_last_active_input_ = true;
    }
    else
    {
      last_active_input_.setZero();
      has_last_active_input_ = false;
    }

    Eigen::Matrix3d frame;
    frame.col(0) = next_x;
    frame.col(1) = next_y;
    frame.col(2) = tool_z;
    return frame;
  }

  Eigen::Vector3d mapAngularInputToBase(const std::string &orientation_frame_id,
                                       const Eigen::Quaterniond &current_orientation,
                                       const Eigen::Vector3d &angular_input,
                                       double cone_angle_rad, double release_threshold,
                                       HybridOrientationFrame &hybrid_frame,
                                       Eigen::Matrix3d *orientation_frame_to_base)
  {
    Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
    if (orientation_frame_id == "effector_frame")
    {
      Eigen::Quaterniond normalized_orientation = current_orientation;
      if (normalized_orientation.norm() < kVectorEpsilon)
      {
        normalized_orientation = Eigen::Quaterniond::Identity();
      }
      else
      {
        normalized_orientation.normalize();
      }
      frame = normalized_orientation.toRotationMatrix();
    }
    else if (orientation_frame_id == "hybrid_frame")
    {
      frame = hybrid_frame.update(current_orientation, angular_input, cone_angle_rad,
                                  release_threshold);
    }

    if (orientation_frame_to_base != nullptr)
    {
      *orientation_frame_to_base = frame;
    }
    return frame * angular_input;
  }
} // namespace manager_core
