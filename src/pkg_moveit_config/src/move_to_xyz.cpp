#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>


static double parse_double(const std::string & value, const std::string & name)
{
  try {
    size_t idx = 0;
    const double result = std::stod(value, &idx);

    if (idx != value.size()) {
      throw std::runtime_error("Trailing characters");
    }

    return result;
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid value for " + name + ": " + value);
  }
}


static void normalize_quaternion(geometry_msgs::msg::Quaternion & q)
{
  const double norm = std::sqrt(
    q.x * q.x +
    q.y * q.y +
    q.z * q.z +
    q.w * q.w
  );

  if (norm < 1e-9) {
    throw std::runtime_error("Quaternion norm is zero.");
  }

  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "move_to_xyz",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  if (args.size() != 4 && args.size() != 8) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Uso: move_to_xyz X Y Z [qx qy qz qw]"
    );
    rclcpp::shutdown();
    return 1;
  }

  double x;
  double y;
  double z;

  try {
    x = parse_double(args[1], "x");
    y = parse_double(args[2], "y");
    z = parse_double(args[3], "z");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  try {
    static const std::string PLANNING_GROUP = "arm";
    static const std::string PLANNING_FRAME = "world";
    static const std::string TCP_LINK = "fp3_hand_tcp";

    moveit::planning_interface::MoveGroupInterface move_group(
      node,
      PLANNING_GROUP
    );

    move_group.setPoseReferenceFrame(PLANNING_FRAME);
    move_group.setEndEffectorLink(TCP_LINK);

    move_group.setStartStateToCurrentState();

    move_group.setPlannerId("RRTConnect");
    move_group.setPlanningTime(15.0);
    move_group.setNumPlanningAttempts(30);

    move_group.setMaxVelocityScalingFactor(0.15);
    move_group.setMaxAccelerationScalingFactor(0.15);

    move_group.setGoalPositionTolerance(0.008);
    move_group.setGoalOrientationTolerance(0.08);
    move_group.setGoalJointTolerance(0.02);

    auto current_pose = move_group.getCurrentPose(TCP_LINK);

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    const bool fixed_orientation_requested =
      args.size() == 8 &&
      args[4] != "keep" &&
      args[5] != "keep" &&
      args[6] != "keep" &&
      args[7] != "keep";

    if (fixed_orientation_requested) {
      target_pose.orientation.x = parse_double(args[4], "qx");
      target_pose.orientation.y = parse_double(args[5], "qy");
      target_pose.orientation.z = parse_double(args[6], "qz");
      target_pose.orientation.w = parse_double(args[7], "qw");

      normalize_quaternion(target_pose.orientation);

      RCLCPP_INFO(
        node->get_logger(),
        "Orientación final fija activada para %s.",
        TCP_LINK.c_str()
      );
    } else {
      target_pose.orientation = current_pose.pose.orientation;

      RCLCPP_WARN(
        node->get_logger(),
        "No se pasó quaternion fijo. Se mantiene la orientación actual."
      );
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Planificando a XYZ: x=%.3f y=%.3f z=%.3f",
      x,
      y,
      z
    );

    RCLCPP_INFO(
      node->get_logger(),
      "Quaternion objetivo: qx=%.4f qy=%.4f qz=%.4f qw=%.4f",
      target_pose.orientation.x,
      target_pose.orientation.y,
      target_pose.orientation.z,
      target_pose.orientation.w
    );

    move_group.clearPoseTargets();
    move_group.clearPathConstraints();

    move_group.setPoseTarget(target_pose, TCP_LINK);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group.plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "La planificación ha fallado.");

      move_group.clearPoseTargets();
      move_group.clearPathConstraints();

      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }

      rclcpp::shutdown();
      return 2;
    }

    RCLCPP_INFO(node->get_logger(), "Planificación correcta. Ejecutando...");

    const auto exec_result = move_group.execute(plan);

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "La ejecución ha fallado.");

      move_group.clearPoseTargets();
      move_group.clearPathConstraints();

      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }

      rclcpp::shutdown();
      return 3;
    }

    auto final_pose = move_group.getCurrentPose(TCP_LINK);

    RCLCPP_INFO(
      node->get_logger(),
      "Pose final TCP: x=%.4f y=%.4f z=%.4f | qx=%.4f qy=%.4f qz=%.4f qw=%.4f",
      final_pose.pose.position.x,
      final_pose.pose.position.y,
      final_pose.pose.position.z,
      final_pose.pose.orientation.x,
      final_pose.pose.orientation.y,
      final_pose.pose.orientation.z,
      final_pose.pose.orientation.w
    );

    RCLCPP_INFO(node->get_logger(), "Ejecución completada correctamente.");

    move_group.clearPoseTargets();
    move_group.clearPathConstraints();

  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Excepción en move_to_xyz: %s", e.what());

    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }

    rclcpp::shutdown();
    return 10;
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}