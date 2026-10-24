#include "robot_arm_core/kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace robot_arm_core
{

PlanarArmKinematics::PlanarArmKinematics(const std::array<double, kDof> & link_lengths)
: link_lengths_(link_lengths)
{
}

Point2D PlanarArmKinematics::forward(const JointArray & q) const
{
  const double a1 = q[0];
  const double a2 = a1 + q[1];
  const double a3 = a2 + q[2];

  return Point2D{
    link_lengths_[0] * std::cos(a1) + link_lengths_[1] * std::cos(a2) +
      link_lengths_[2] * std::cos(a3),
    link_lengths_[0] * std::sin(a1) + link_lengths_[1] * std::sin(a2) +
      link_lengths_[2] * std::sin(a3)};
}

std::array<std::array<double, 3>, 2> PlanarArmKinematics::jacobian(
  const JointArray & q) const
{
  // dP/dq_i summed over the links distal to joint i.
  const double a1 = q[0];
  const double a2 = a1 + q[1];
  const double a3 = a2 + q[2];

  std::array<std::array<double, 3>, 2> J{};
  J[0][0] = -link_lengths_[0] * std::sin(a1) -
    link_lengths_[1] * std::sin(a2) - link_lengths_[2] * std::sin(a3);
  J[1][0] = link_lengths_[0] * std::cos(a1) +
    link_lengths_[1] * std::cos(a2) + link_lengths_[2] * std::cos(a3);
  J[0][1] = -link_lengths_[1] * std::sin(a2) - link_lengths_[2] * std::sin(a3);
  J[1][1] = link_lengths_[1] * std::cos(a2) + link_lengths_[2] * std::cos(a3);
  J[0][2] = -link_lengths_[2] * std::sin(a3);
  J[1][2] = link_lengths_[2] * std::cos(a3);
  return J;
}

bool PlanarArmKinematics::inverse(
  const Point2D & target,
  JointArray & joint_angles,
  const JointArray & seed,
  int max_iterations) const
{
  if (!is_reachable(target)) {
    return false;
  }

  JointArray q = apply_joint_limits(seed);

  constexpr double kTolerance = 1e-10;
  constexpr double kDamping = 1e-6;   // Levenberg-Marquardt damping for singularities
  constexpr double kMaxStep = 0.5;    // radians per iteration

  for (int iter = 0; iter < max_iterations; ++iter) {
    const Point2D current = forward(q);
    const double ex = target.x - current.x;
    const double ey = target.y - current.y;
    const double error_norm = std::hypot(ex, ey);

    if (error_norm < kTolerance) {
      joint_angles = q;
      return true;
    }

    const auto J = jacobian(q);

    // Solve the damped least-squares system J * dq = e.
    // J is 2x3; the pseudo-inverse is J^T (J J^T + lambda I)^-1.
    const double a = J[0][0] * J[0][0] + J[0][1] * J[0][1] + J[0][2] * J[0][2] + kDamping;
    const double b = J[0][0] * J[1][0] + J[0][1] * J[1][1] + J[0][2] * J[1][2];
    const double c = J[1][0] * J[1][0] + J[1][1] * J[1][1] + J[1][2] * J[1][2] + kDamping;

    const double det = a * c - b * b;
    if (std::abs(det) < 1e-14) {
      // Fully singular configuration - cannot proceed.
      return false;
    }

    const double inv_00 = c / det;
    const double inv_01 = -b / det;
    const double inv_11 = a / det;

    // (J J^T)^-1 * e
    const double y0 = inv_00 * ex + inv_01 * ey;
    const double y1 = inv_01 * ex + inv_11 * ey;

    // dq = J^T * y
    double dq0 = J[0][0] * y0 + J[1][0] * y1;
    double dq1 = J[0][1] * y0 + J[1][1] * y1;
    double dq2 = J[0][2] * y0 + J[1][2] * y1;

    // Limit the step size for stability.
    const double step_norm = std::abs(dq0) + std::abs(dq1) + std::abs(dq2);
    if (step_norm > kMaxStep) {
      const double scale = kMaxStep / step_norm;
      dq0 *= scale;
      dq1 *= scale;
      dq2 *= scale;
    }

    // Backtracking line search: halve the step until the error decreases.
    double scale = 1.0;
    bool improved = false;
    for (int ls = 0; ls < 30; ++ls) {
      const JointArray candidate{
        q[0] + scale * dq0,
        q[1] + scale * dq1,
        q[2] + scale * dq2};
      const Point2D p = forward(candidate);
      if (std::hypot(target.x - p.x, target.y - p.y) < error_norm) {
        q = apply_joint_limits(candidate);
        improved = true;
        break;
      }
      scale *= 0.5;
    }

    if (!improved) {
      // No downhill direction found; stop and report the best we have.
      const Point2D p = forward(q);
      if (std::hypot(target.x - p.x, target.y - p.y) < 1e-6) {
        joint_angles = q;
        return true;
      }
      return false;
    }
  }

  // Ran out of iterations - accept if we are close enough.
  const Point2D p = forward(q);
  if (std::hypot(target.x - p.x, target.y - p.y) < 1e-7) {
    joint_angles = q;
    return true;
  }
  return false;
}

bool PlanarArmKinematics::inverseWithFixedQ3(
  const Point2D & target,
  double q3_fixed,
  JointArray & joint_angles,
  bool elbow_up) const
{
  // With q3 fixed, the arm is a planar 2-link chain with an end-effector
  // offset of l3 at a known angle relative to link 2.
  //
  //   x = l1*cos(a1) + l2*cos(a1+a2) + l3*cos(a1+a2+q3)
  //   y = l1*sin(a1) + l2*sin(a1+a2) + l3*sin(a1+a2+q3)
  //
  // Treat (l2, l3) as a single effective link of the "2R + offset" form:
  // the wrist point W = P - l3*(cos(a1+a2+q3), sin(a1+a2+q3)) is reached
  // by the 2-link chain (l1, l2). We therefore solve the 2-link IK for the
  // wrist point, iterating once because W depends on a1+a2.
  //
  // Because q3 is fixed and l2, l3 are known, we can instead offset the
  // target by rotating it back by q3 around the (a1+a2) direction. The
  // cleanest exact approach is the geometric two-circle method:
  //   Circle A: |E1| = l1          (joint 1 position)
  //   Circle B: |W  - E1| = l2     (wrist point relative to joint 1)
  //   Circle C: |P  - W|  = l3     (tool offset, direction known by q3)
  // Solving A and B gives E1 and the angle a1+a2; then W follows and C
  // is satisfied by construction when the target is consistent.

  // ---- Step 1: solve the 2-link chain (l1, l2) for the "virtual" target ----
  // The tool offset is rigidly attached to link 2, so the wrist point W is
  // obtained by rotating the target by -q3 about the wrist joint. We do this
  // with an iteration: guess a1+a2, compute W, solve 2-link IK, repeat.
  double phi = 0.0;  // initial guess for a1 + a2
  JointArray q{};

  for (int iter = 0; iter < 50; ++iter) {
    // Wrist point given the current guess of the link-2 direction.
    const double a2_dir = phi;
    const double wx = target.x - link_lengths_[2] * std::cos(a2_dir + q3_fixed);
    const double wy = target.y - link_lengths_[2] * std::sin(a2_dir + q3_fixed);

    // 2-link IK for (l1, l2) reaching (wx, wy).
    const double r = std::hypot(wx, wy);
    if (r < 1e-12 || r > link_lengths_[0] + link_lengths_[1] ||
      r < std::abs(link_lengths_[0] - link_lengths_[1]))
    {
      return false;
    }

    const double c2 = (r * r - link_lengths_[0] * link_lengths_[0] -
                 link_lengths_[1] * link_lengths_[1]) /
              (2.0 * link_lengths_[0] * link_lengths_[1]);
    if (c2 < -1.0 || c2 > 1.0) {
      return false;
    }
    const double s2 = elbow_up ? std::sqrt(1.0 - c2 * c2) : -std::sqrt(1.0 - c2 * c2);
    const double q2 = std::atan2(s2, c2);
    const double q1 = std::atan2(wy, wx) -
      std::atan2(link_lengths_[1] * s2, link_lengths_[0] + link_lengths_[1] * c2);

    q = {q1, q2, q3_fixed};

    const double new_phi = q1 + q2;
    if (std::abs(new_phi - phi) < 1e-14) {
      phi = new_phi;
      break;
    }
    phi = new_phi;
  }

  // ---- Verify the solution ----
  const Point2D p = forward(q);
  if (std::hypot(target.x - p.x, target.y - p.y) > 1e-8) {
    return false;
  }

  joint_angles = apply_joint_limits(q);
  return true;
}

double PlanarArmKinematics::max_reach() const
{
  return link_lengths_[0] + link_lengths_[1] + link_lengths_[2];
}

double PlanarArmKinematics::min_reach() const
{
  const double total = max_reach();
  const double longest = *std::max_element(link_lengths_.begin(), link_lengths_.end());
  return std::max(0.0, longest - (total - longest));
}

bool PlanarArmKinematics::is_reachable(const Point2D & target) const
{
  const double r = std::hypot(target.x, target.y);
  return r <= max_reach() + 1e-9 && r >= min_reach() - 1e-9;
}

double PlanarArmKinematics::normalize_angle(double angle)
{
  constexpr double kTwoPi = 2.0 * M_PI;
  angle = std::fmod(angle + M_PI, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - M_PI;
}

JointArray PlanarArmKinematics::apply_joint_limits(const JointArray & joint_angles) const
{
  JointArray out{};
  for (std::size_t i = 0; i < kDof; ++i) {
    out[i] = std::clamp(joint_angles[i], joint_lower_[i], joint_upper_[i]);
  }
  return out;
}

void PlanarArmKinematics::set_joint_limits(const JointArray & lower, const JointArray & upper)
{
  joint_lower_ = lower;
  joint_upper_ = upper;
}

}  // namespace robot_arm_core
