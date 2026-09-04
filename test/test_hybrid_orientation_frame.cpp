#include <cmath>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "cartesian_manager/core/hybrid_orientation_frame.hpp"

namespace
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kConeAngle = 5.0 * kPi / 180.0;
  constexpr double kReleaseThreshold = 0.03;
  constexpr double kTolerance = 1e-9;

  Eigen::Quaterniond orientationFromToolZ(const Eigen::Vector3d &tool_z)
  {
    return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), tool_z.normalized());
  }

  void expectOrthonormal(const Eigen::Matrix3d &frame)
  {
    EXPECT_TRUE(frame.transpose().isApprox(frame.inverse(), kTolerance));
    EXPECT_NEAR(frame.determinant(), 1.0, kTolerance);
  }
} // namespace

TEST(HybridOrientationFrame, ConstructsPaperFrameForHorizontalTool)
{
  manager_core::HybridOrientationFrame hybrid_frame;
  const Eigen::Matrix3d frame = hybrid_frame.update(
      orientationFromToolZ(Eigen::Vector3d::UnitX()), Eigen::Vector3d::Zero(), kConeAngle,
      kReleaseThreshold);

  EXPECT_TRUE(frame.col(0).isApprox(Eigen::Vector3d::UnitY(), kTolerance));
  EXPECT_TRUE(frame.col(1).isApprox(Eigen::Vector3d::UnitZ(), kTolerance));
  EXPECT_TRUE(frame.col(2).isApprox(Eigen::Vector3d::UnitX(), kTolerance));
  expectOrthonormal(frame);
}

TEST(HybridOrientationFrame, UsesOrthonormalFallbackAtBothVerticalPoles)
{
  manager_core::HybridOrientationFrame hybrid_frame;
  const Eigen::Matrix3d upward = hybrid_frame.update(
      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), kConeAngle, kReleaseThreshold);
  EXPECT_TRUE(upward.isApprox(Eigen::Matrix3d::Identity(), kTolerance));

  hybrid_frame.reset();
  const Eigen::Matrix3d downward = hybrid_frame.update(
      orientationFromToolZ(-Eigen::Vector3d::UnitZ()), Eigen::Vector3d::Zero(), kConeAngle,
      kReleaseThreshold);
  EXPECT_TRUE(downward.col(0).isApprox(Eigen::Vector3d::UnitX(), kTolerance));
  EXPECT_TRUE(downward.col(1).isApprox(-Eigen::Vector3d::UnitY(), kTolerance));
  EXPECT_TRUE(downward.col(2).isApprox(-Eigen::Vector3d::UnitZ(), kTolerance));
  expectOrthonormal(downward);
}

TEST(HybridOrientationFrame, PreservesAxisContinuityUntilRelease)
{
  manager_core::HybridOrientationFrame hybrid_frame;
  const Eigen::Vector3d active_input = Eigen::Vector3d::UnitX();
  const double outside_angle = 10.0 * kPi / 180.0;
  const double inside_angle = 1.0 * kPi / 180.0;
  const Eigen::Vector3d before_z(std::sin(outside_angle), 0.0, std::cos(outside_angle));
  const Eigen::Vector3d inside_z(std::sin(inside_angle), 0.0, std::cos(inside_angle));
  const Eigen::Vector3d after_z(-std::sin(outside_angle), 0.0, std::cos(outside_angle));

  const Eigen::Matrix3d before = hybrid_frame.update(
      orientationFromToolZ(before_z), active_input, kConeAngle, kReleaseThreshold);
  const Eigen::Matrix3d inside = hybrid_frame.update(
      orientationFromToolZ(inside_z), active_input, kConeAngle, kReleaseThreshold);
  const Eigen::Matrix3d after = hybrid_frame.update(
      orientationFromToolZ(after_z), active_input, kConeAngle, kReleaseThreshold);

  EXPECT_GT(before.col(0).dot(inside.col(0)), 0.0);
  EXPECT_GT(inside.col(0).dot(after.col(0)), 0.0);
  EXPECT_GT(after.col(0).dot(Eigen::Vector3d::UnitY()), 0.0);

  const Eigen::Matrix3d released = hybrid_frame.update(
      orientationFromToolZ(after_z), Eigen::Vector3d::Zero(), kConeAngle, kReleaseThreshold);
  EXPECT_LT(released.col(0).dot(Eigen::Vector3d::UnitY()), 0.0);
}

TEST(HybridOrientationFrame, ReversalReanchorsInsideCone)
{
  manager_core::HybridOrientationFrame hybrid_frame;
  const double outside_angle = 10.0 * kPi / 180.0;
  const double inside_angle = 1.0 * kPi / 180.0;
  const Eigen::Vector3d outside_z(std::sin(outside_angle), 0.0, std::cos(outside_angle));
  const Eigen::Vector3d inside_z(std::sin(inside_angle), 0.0, std::cos(inside_angle));

  hybrid_frame.update(orientationFromToolZ(outside_z), Eigen::Vector3d::UnitX(), kConeAngle,
                      kReleaseThreshold);
  const Eigen::Matrix3d reversed = hybrid_frame.update(
      orientationFromToolZ(inside_z), -Eigen::Vector3d::UnitX(), kConeAngle,
      kReleaseThreshold);

  EXPECT_GT(reversed.col(0).dot(Eigen::Vector3d::UnitX()), 0.99);
  expectOrthonormal(reversed);
}

TEST(HybridOrientationFrame, RotationPreservesInputNorm)
{
  manager_core::HybridOrientationFrame hybrid_frame;
  const Eigen::Vector3d input(0.2, -0.4, 0.7);
  const Eigen::Matrix3d frame = hybrid_frame.update(
      orientationFromToolZ(Eigen::Vector3d(0.4, 0.3, 0.8)), input, kConeAngle,
      kReleaseThreshold);

  EXPECT_NEAR((frame * input).norm(), input.norm(), kTolerance);
  expectOrthonormal(frame);
}

TEST(HybridOrientationFrame, MapsAllOrientationFramesToBase)
{
  manager_core::HybridOrientationFrame hybrid_frame;
  const Eigen::Vector3d input = Eigen::Vector3d::UnitX();
  const Eigen::Quaterniond quarter_turn(
      Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d base_frame;
  Eigen::Matrix3d effector_frame;
  Eigen::Matrix3d adaptive_frame;

  const Eigen::Vector3d base = manager_core::mapAngularInputToBase(
      "base_frame", quarter_turn, input, kConeAngle, kReleaseThreshold, hybrid_frame,
      &base_frame);
  const Eigen::Vector3d effector = manager_core::mapAngularInputToBase(
      "effector_frame", quarter_turn, input, kConeAngle, kReleaseThreshold, hybrid_frame,
      &effector_frame);
  const Eigen::Vector3d hybrid = manager_core::mapAngularInputToBase(
      "hybrid_frame", Eigen::Quaterniond::Identity(), input, kConeAngle, kReleaseThreshold,
      hybrid_frame, &adaptive_frame);

  EXPECT_TRUE(base.isApprox(Eigen::Vector3d::UnitX(), kTolerance));
  EXPECT_TRUE(effector.isApprox(Eigen::Vector3d::UnitY(), kTolerance));
  EXPECT_TRUE(hybrid.isApprox(Eigen::Vector3d::UnitX(), kTolerance));
  EXPECT_TRUE(base_frame.isApprox(Eigen::Matrix3d::Identity(), kTolerance));
  EXPECT_TRUE(effector_frame.isApprox(quarter_turn.toRotationMatrix(), kTolerance));
  EXPECT_TRUE(adaptive_frame.isApprox(Eigen::Matrix3d::Identity(), kTolerance));
}

TEST(HybridOrientationFrame, ResetDropsContinuityState)
{
  manager_core::HybridOrientationFrame hybrid_frame;
  const double outside_angle = 10.0 * kPi / 180.0;
  const Eigen::Vector3d first_z(std::sin(outside_angle), 0.0, std::cos(outside_angle));
  const Eigen::Vector3d second_z(-std::sin(outside_angle), 0.0, std::cos(outside_angle));

  hybrid_frame.update(orientationFromToolZ(first_z), Eigen::Vector3d::UnitX(), kConeAngle, 0.0);
  const Eigen::Matrix3d continuous = hybrid_frame.update(
      orientationFromToolZ(second_z), Eigen::Vector3d::UnitX(), kConeAngle, 0.0);
  hybrid_frame.reset();
  const Eigen::Matrix3d reset = hybrid_frame.update(
      orientationFromToolZ(second_z), Eigen::Vector3d::UnitX(), kConeAngle, 0.0);

  EXPECT_GT(continuous.col(0).dot(Eigen::Vector3d::UnitY()), 0.0);
  EXPECT_LT(reset.col(0).dot(Eigen::Vector3d::UnitY()), 0.0);
}
