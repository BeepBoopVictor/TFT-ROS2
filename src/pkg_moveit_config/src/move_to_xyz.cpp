#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;

static double parse_double(const char * value, const std::string & name)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid value for " + name + ": " + value);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "move_to_xyz",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  if (argc < 4) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Usage: ros2 run pkg_moveit_config move_to_xyz X Y Z [qx qy qz qw]"
    );
    rclcpp::shutdown();
    return 1;
  }

  const double x = parse_double(argv[1], "x");
  const double y = parse_double(argv[2], "y");
  const double z = parse_double(argv[3], "z");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPoseReferenceFrame("world");
  move_group.setEndEffectorLink("fp3_hand_tcp");

  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.25);
  move_group.setMaxAccelerationScalingFactor(0.25);

  auto current_pose = move_group.getCurrentPose("fp3_hand_tcp");

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  if (argc >= 8) {
    target_pose.orientation.x = parse_double(argv[4], "qx");
    target_pose.orientation.y = parse_double(argv[5], "qy");
    target_pose.orientation.z = parse_double(argv[6], "qz");
    target_pose.orientation.w = parse_double(argv[7], "qw");
  } else {
    // Por defecto mantiene la orientación actual del TCP.
    // Esto facilita la primera validación XYZ sin pelear todavía con IK de orientación.
    target_pose.orientation = current_pose.pose.orientation;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Planning to XYZ: x=%.3f y=%.3f z=%.3f",
    x, y, z
  );

  move_group.setPoseTarget(target_pose, "fp3_hand_tcp");

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  const auto plan_result = move_group.plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 2;
  }

  RCLCPP_INFO(node->get_logger(), "Planning succeeded. Executing...");

  const auto exec_result = move_group.execute(plan);

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed.");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 3;
  }

  RCLCPP_INFO(node->get_logger(), "Execution succeeded.");

  move_group.clearPoseTargets();

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}