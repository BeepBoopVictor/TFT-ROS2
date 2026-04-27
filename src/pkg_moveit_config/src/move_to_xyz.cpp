#include <chrono>
#include <cstdlib>
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "move_to_xyz",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  // args[0] es el nombre del ejecutable.
  // args válidos:
  //   move_to_xyz X Y Z
  //   move_to_xyz X Y Z qx qy qz qw
  if (args.size() != 4 && args.size() != 8) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Uso: ros2 launch pkg_moveit_config demo_xyz.launch.py x:=0.45 y:=0.00 z:=0.55"
    );
    RCLCPP_ERROR(
      node->get_logger(),
      "O directo con parámetros cargados: move_to_xyz X Y Z [qx qy qz qw]"
    );
    rclcpp::shutdown();
    return 1;
  }

  double x, y, z;

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

    moveit::planning_interface::MoveGroupInterface move_group(
      node,
      PLANNING_GROUP
    );

    move_group.setPoseReferenceFrame("world");
    move_group.setEndEffectorLink("fp3_hand_tcp");

    move_group.setPlanningTime(8.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.20);
    move_group.setMaxAccelerationScalingFactor(0.20);

    auto current_pose = move_group.getCurrentPose("fp3_hand_tcp");

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    if (args.size() == 8) {
      target_pose.orientation.x = parse_double(args[4], "qx");
      target_pose.orientation.y = parse_double(args[5], "qy");
      target_pose.orientation.z = parse_double(args[6], "qz");
      target_pose.orientation.w = parse_double(args[7], "qw");
    } else {
      // Mantiene la orientación actual del TCP.
      // Esto hace que el test XYZ sea más estable al principio.
      target_pose.orientation = current_pose.pose.orientation;
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
      "Orientación objetivo: qx=%.4f qy=%.4f qz=%.4f qw=%.4f",
      target_pose.orientation.x,
      target_pose.orientation.y,
      target_pose.orientation.z,
      target_pose.orientation.w
    );

    move_group.setPoseTarget(target_pose, "fp3_hand_tcp");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    const auto plan_result = move_group.plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "La planificación ha fallado.");
      move_group.clearPoseTargets();

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

      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }

      rclcpp::shutdown();
      return 3;
    }

    RCLCPP_INFO(node->get_logger(), "Ejecución completada correctamente.");

    move_group.clearPoseTargets();

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