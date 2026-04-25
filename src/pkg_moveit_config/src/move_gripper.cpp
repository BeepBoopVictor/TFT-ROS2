#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

static constexpr double MIN_FINGER_POS = 0.003;
static constexpr double MAX_FINGER_POS = 0.040;

static double clamp_width(double width)
{
  if (width <= 0.0) {
    return MIN_FINGER_POS;
  }
  return std::max(MIN_FINGER_POS, std::min(MAX_FINGER_POS, width));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "move_gripper",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  double width = MAX_FINGER_POS;
  double duration = 2.0;

  if (argc >= 2) {
    width = std::stod(argv[1]);
  }

  if (argc >= 3) {
    duration = std::stod(argv[2]);
  }

  width = clamp_width(width);
  duration = std::max(0.8, duration);

  auto client = rclcpp_action::create_client<FollowJointTrajectory>(
    node,
    "/fp3_hand_controller/follow_joint_trajectory"
  );

  RCLCPP_INFO(node->get_logger(), "Waiting for gripper action server...");

  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available.");
    rclcpp::shutdown();
    return 1;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {
    "fp3_finger_joint1",
    "fp3_finger_joint2",
  };

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {width, width};
  point.velocities = {0.0, 0.0};

  builtin_interfaces::msg::Duration time_from_start;
  time_from_start.sec = static_cast<int32_t>(duration);
  time_from_start.nanosec = static_cast<uint32_t>((duration - time_from_start.sec) * 1e9);

  point.time_from_start = time_from_start;
  goal.trajectory.points.push_back(point);

  RCLCPP_INFO(
    node->get_logger(),
    "Sending gripper command width=%.4f duration=%.2f",
    width,
    duration
  );

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

  auto goal_handle_future = client->async_send_goal(goal, send_goal_options);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send gripper goal.");
    rclcpp::shutdown();
    return 2;
  }

  auto goal_handle = goal_handle_future.get();

  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Gripper goal was rejected.");
    rclcpp::shutdown();
    return 3;
  }

  auto result_future = client->async_get_result(goal_handle);

  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get gripper result.");
    rclcpp::shutdown();
    return 4;
  }

  RCLCPP_INFO(node->get_logger(), "Gripper command completed.");

  rclcpp::shutdown();
  return 0;
}