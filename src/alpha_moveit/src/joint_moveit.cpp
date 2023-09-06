#include <memory>

#include "std_srvs/srv/trigger.hpp"

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
  //-----------------------------------------------------
  rclcpp::init(argc, argv);


  auto const node = std::make_shared<rclcpp::Node>(
      "joint_hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Initialising and defining the planning group for move_base
  static const std::string PLANNING_GROUP = "alpha";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // A pointer that references the current robot's state.
  moveit::core::RobotStatePtr current_state(new moveit::core::RobotState(move_group.getRobotModel()));

  // Set the joint names and values for the current state
  std::vector<std::string> joint_names = {"axis_a", "axis_b", "axis_c", "axis_d", "axis_e"};
  std::vector<double> joint_values = {0.0, 1.5746, 0.0, 1.5746, 0.0};


  // Print the current joint values
  RCLCPP_INFO(logger, "Current joint values");
  // for (size_t i = 0; i < joint_values.size(); ++i)
  // {
  //   RCLCPP_INFO(logger, "Joint %zu: %.2f", i, joint_values[i]);
  // }

  // Set the joint positions in the current state
  current_state->setJointGroupPositions(move_group.getName(), joint_values);

  // Set the current state in the MoveGroupInterface
  move_group.setStartState(*current_state);
  
  // Get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  joint_group_positions = joint_values;

  // Modifying one of the joint positions
  // joint_group_positions[0] = 0.0;    // m
  // joint_group_positions[1] = 0.0;    // radians
  // joint_group_positions[2] = 0.0;    // radians
  joint_group_positions[2] = joint_values[2] + 0.5; // radians
  // joint_group_positions[4] = 0.0;    // radians

  // Pass the desired joint positions to move_group as goal for planning
  move_group.setJointValueTarget(joint_group_positions);

  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  move_group.execute(my_plan);

  std::this_thread::sleep_for(std::chrono::seconds(10));

  rclcpp::shutdown();
  return 0;
}
