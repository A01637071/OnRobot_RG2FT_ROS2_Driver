#ifndef ONROBOT_RG2FT_ACTION_SERVER_H
#define ONROBOT_RG2FT_ACTION_SERVER_H

// STL
#include <string>
#include <memory>
#include <functional>
#include <atomic>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Actions / msgs
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "onrobot_rg2ft_msgs/msg/on_robot_rg2_ft_data.hpp"
#include "onrobot_rg2ft_msgs/msg/rg2_ft_command.hpp"

namespace onrobot_rg2ft_action_server
{

// DH gripper control parameters
#define MIN_POSITION 0
#define MAX_POSITION 1000
#define MIN_FORCE 30
#define MAX_FORCE 400
#define GOAL_TOLERANCE 20

using GripperCtrl = onrobot_rg2ft_msgs::msg::RG2FTCommand;
using GripperState = onrobot_rg2ft_msgs::msg::OnRobotRG2FTData;

using GripperCommand = control_msgs::action::GripperCommand;
using GripperCommandGoal = GripperCommand::Goal;
using GripperCommandFeedback = GripperCommand::Feedback;
using GripperCommandResult = GripperCommand::Result;

struct BadArgumentsError {};

/**
 * @brief Structure containing the parameters necessary to translate
 *        GripperCommand actions to register-based commands to a
 *        particular gripper (and vice versa).
 */
struct GripperParams
{
  double min_angle_; // radians
  double max_angle_;
  double min_effort_; // N / (Nm)
  double max_effort_;
  double default_effort_;
  std::string control_topic_;
  std::string state_topic_;
  std::string joint_states_topic_;
  std::string joint_name_;
};

/**
 * @brief The OnRobotRG2FTActionServer node class.
 */
class OnRobotRG2FTActionServer : public rclcpp::Node, public std::enable_shared_from_this<OnRobotRG2FTActionServer>
{
public:
  using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperCommand>;

  // ✅ Constructor corregido: recibe node_name y action_name por separado
  OnRobotRG2FTActionServer(
    const std::string & node_name,
    const GripperParams & params,
    const std::string & action_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Action callbacks (rclcpp_action style)
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GripperCommandGoal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGripper> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle);

  // State subscription callback (gripper publishes state messages)
  void state_callback(const GripperState::SharedPtr msg);

private:
  // Helpers
  GripperCtrl goalToGripperCtrl(const GripperCommandGoal & goal);
  void issueInitialization();
  void publishJointStates(const GripperState::SharedPtr gripper_state);
  double mapRange(double val, double prev_min, double prev_max, double new_min, double new_max, bool reverse);

  // Action server
  rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;

  // Pub / subs
  rclcpp::Subscription<GripperState>::SharedPtr state_sub_;
  rclcpp::Publisher<GripperCtrl>::SharedPtr goal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

  // Parameters & state
  std::atomic<int> position_goal_;
  std::atomic<int> force_goal_;
  std::atomic<bool> is_initialized_;

  GripperParams gripper_params_;
  std::string action_name_;
};

} // namespace onrobot_rg2ft_action_server

#endif // ONROBOT_RG2FT_ACTION_SERVER_H

