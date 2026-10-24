#ifndef ROBOT_ARM_CORE__KINEMATICS_HPP_
#define ROBOT_ARM_CORE__KINEMATICS_HPP_

#include <array>
#include <cmath>
#include <optional>

namespace robot_arm_core
{

/// Number of revolute joints in the planar arm.
inline constexpr std::size_t kDof = 3;

/// Joint configuration: three revolute joint angles in radians.
using JointArray = std::array<double, kDof>;

/// A 2-D Cartesian point.
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

/// Planar 3-DOF arm kinematics with forward and inverse kinematics.
///
/// The arm consists of three revolute joints whose axes are parallel to Z.
/// Link lengths are in metres; joint angles are in radians and follow the
/// right-hand rule about +Z.
class PlanarArmKinematics
{
public:
  /// Construct with the link lengths in metres.
  explicit PlanarArmKinematics(const std::array<double, kDof> & link_lengths);

  /// Forward kinematics: joint angles -> end-effector position.
  [[nodiscard]] Point2D forward(const JointArray & joint_angles) const;

  /// Inverse kinematics: target position -> joint angles.
  ///
  /// Uses a damped Newton-Raphson iteration on the position Jacobian,
  /// which converges to machine precision for every reachable target and
  /// degrades gracefully near singular configurations.
  ///
  /// \param target         desired end-effector position.
  /// \param joint_angles   receives the solution on success.
  /// \param seed           initial guess for the iteration.
  /// \param max_iterations iteration cap.
  /// \return true on success (joint_angles is filled in).
  [[nodiscard]] bool inverse(
    const Point2D & target,
    JointArray & joint_angles,
    const JointArray & seed = JointArray{0.0, 0.6, 0.6},
    int max_iterations = 200) const;

  /// Inverse kinematics with the joint-3 angle held fixed.
  ///
  /// This reduces the problem to a planar 2-link chain, which has an
  /// exact closed-form solution.
  ///
  /// \param target       desired end-effector position.
  /// \param q3_fixed     joint-3 angle to hold constant (radians).
  /// \param joint_angles receives the solution on success.
  /// \param elbow_up     select the elbow-up (true) or elbow-down branch.
  [[nodiscard]] bool inverseWithFixedQ3(
    const Point2D & target,
    double q3_fixed,
    JointArray & joint_angles,
    bool elbow_up = true) const;

  /// Total reach of the arm (sum of all link lengths).
  [[nodiscard]] double max_reach() const;

  /// Minimum reach (largest link minus the sum of the others).
  [[nodiscard]] double min_reach() const;

  /// Whether a target point lies within the reachable workspace.
  [[nodiscard]] bool is_reachable(const Point2D & target) const;

  /// The 2x3 position Jacobian at the given joint configuration.
  [[nodiscard]] std::array<std::array<double, 3>, 2> jacobian(
    const JointArray & joint_angles) const;

  /// Wrap an angle into the range (-pi, pi].
  [[nodiscard]] static double normalize_angle(double angle);

  /// Constrain joint angles to their configured limits.
  [[nodiscard]] JointArray apply_joint_limits(const JointArray & joint_angles) const;

  /// Set the joint limits in radians.
  void set_joint_limits(const JointArray & lower, const JointArray & upper);

private:
  std::array<double, kDof> link_lengths_;
  JointArray joint_lower_{{-M_PI, -M_PI, -M_PI}};
  JointArray joint_upper_{{M_PI, M_PI, M_PI}};
};

}  // namespace robot_arm_core

#endif  // ROBOT_ARM_CORE__KINEMATICS_HPP_
