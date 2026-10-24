#include "robot_arm_core/kinematics.hpp"

#include <array>
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using robot_arm_core::JointArray;
using robot_arm_core::PlanarArmKinematics;
using robot_arm_core::Point2D;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("arm_node");

  const PlanarArmKinematics arm({0.30, 0.25, 0.15});

  node->declare_parameter<double>("target_x", 0.40);
  node->declare_parameter<double>("target_y", 0.25);

  const double tx = node->get_parameter("target_x").as_double();
  const double ty = node->get_parameter("target_y").as_double();
  const Point2D target{tx, ty};

  RCLCPP_INFO(
    node->get_logger(), "Arm: l1=0.30 l2=0.25 l3=0.15, reach = [%.3f, %.3f]",
    arm.min_reach(), arm.max_reach());

  if (!arm.is_reachable(target)) {
    RCLCPP_WARN(
      node->get_logger(),
      "Target (%.3f, %.3f) is OUT OF REACH (distance %.3f)", tx, ty,
      std::hypot(tx, ty));
    rclcpp::shutdown();
    return 1;
  }

  JointArray q{};
  if (!arm.inverse(target, q)) {
    RCLCPP_ERROR(node->get_logger(), "IK failed for target (%.3f, %.3f)", tx, ty);
    rclcpp::shutdown();
    return 1;
  }

  const Point2D check = arm.forward(q);
  RCLCPP_INFO(
    node->get_logger(),
    "IK solution: q = [%.4f, %.4f, %.4f] rad", q[0], q[1], q[2]);
  RCLCPP_INFO(
    node->get_logger(),
    "FK check   : (%.6f, %.6f)  (target %.6f, %.6f)  err = %.2e",
    check.x, check.y, tx, ty, std::hypot(check.x - tx, check.y - ty));

  rclcpp::shutdown();
  return 0;
}
