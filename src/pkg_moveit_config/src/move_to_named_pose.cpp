#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "move_to_named_pose",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  const std::string target_name = argc >= 2 ? argv[1] : "home";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.25);
  move_group.setMaxAccelerationScalingFactor(0.25);

  RCLCPP_INFO(node->get_logger(), "Moving to named pose: %s", target_name.c_str());

  move_group.setNamedTarget(target_name);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_result = move_group.plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed for named pose: %s", target_name.c_str());
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  const auto exec_result = move_group.execute(plan);

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed for named pose: %s", target_name.c_str());
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 2;
  }

  RCLCPP_INFO(node->get_logger(), "Named pose execution succeeded.");

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}