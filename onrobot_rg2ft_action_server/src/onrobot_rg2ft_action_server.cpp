#include "onrobot_rg2ft_action_server/onrobot_rg2ft_action_server.h"

namespace onrobot_rg2ft_action_server
{

OnRobotRG2FTActionServer::OnRobotRG2FTActionServer(
    const std::string & node_name,
    const GripperParams & params,
    const std::string & action_name,
    const rclcpp::NodeOptions & options)
  : Node(node_name, options),
    action_name_(action_name),
    gripper_params_(params),
    is_initialized_(false),
    position_goal_(0),
    force_goal_(0)
{
  using namespace std::placeholders;

  // Action server
  action_server_ = rclcpp_action::create_server<GripperCommand>(
      this,
      action_name_,
      std::bind(&OnRobotRG2FTActionServer::handle_goal, this, _1, _2),
      std::bind(&OnRobotRG2FTActionServer::handle_cancel, this, _1),
      std::bind(&OnRobotRG2FTActionServer::handle_accepted, this, _1));

  // Pub/Sub
  state_sub_ = this->create_subscription<GripperState>(
      gripper_params_.state_topic_, 10,
      std::bind(&OnRobotRG2FTActionServer::state_callback, this, _1));

  goal_pub_ = this->create_publisher<GripperCtrl>(gripper_params_.control_topic_, 10);
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      gripper_params_.joint_states_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "OnRobot RG2FT action server ready: %s", node_name.c_str());
}


rclcpp_action::GoalResponse
OnRobotRG2FTActionServer::handle_goal(const rclcpp_action::GoalUUID &,
                                      std::shared_ptr<const GripperCommandGoal> goal)
{
  if (goal->command.position < gripper_params_.min_angle_ ||
      goal->command.position > gripper_params_.max_angle_) {
    RCLCPP_WARN(this->get_logger(), "Goal position out of range");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "Accepted new goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
OnRobotRG2FTActionServer::handle_cancel(const std::shared_ptr<GoalHandleGripper> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Cancel requested");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OnRobotRG2FTActionServer::handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle)
{
  std::thread([this, goal_handle]() {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GripperCommandFeedback>();
    auto result = std::make_shared<GripperCommandResult>();

    try {
      auto ctrl_msg = goalToGripperCtrl(*goal);
      goal_pub_->publish(ctrl_msg);
      position_goal_ = ctrl_msg.target_width;
      force_goal_ = ctrl_msg.target_force;
    } catch (BadArgumentsError &) {
      RCLCPP_WARN(this->get_logger(), "Invalid goal arguments");
      goal_handle->abort(result);
      return;
    }

    rclcpp::Rate rate(20);
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        goal_handle->canceled(result);
        return;
      }
      rate.sleep();
    }

    goal_handle->succeed(result);
  }).detach();
}

void OnRobotRG2FTActionServer::state_callback(const GripperState::SharedPtr msg)
{
  publishJointStates(msg);
}

GripperCtrl OnRobotRG2FTActionServer::goalToGripperCtrl(const GripperCommandGoal & goal)
{
  double angle = goal.command.position;
  double max_effort = goal.command.max_effort == 0 ? gripper_params_.default_effort_
                                                   : goal.command.max_effort;

  if (angle < gripper_params_.min_angle_ || angle > gripper_params_.max_angle_ ||
      max_effort < gripper_params_.min_effort_ || max_effort > gripper_params_.max_effort_)
    throw BadArgumentsError();

  double gripper_ctrl_position = mapRange(angle,
                                          gripper_params_.min_angle_,
                                          gripper_params_.max_angle_,
                                          MIN_POSITION,
                                          MAX_POSITION,
                                          true);

  double gripper_ctrl_effort = mapRange(max_effort,
                                        gripper_params_.min_effort_,
                                        gripper_params_.max_effort_,
                                        MIN_FORCE,
                                        MAX_FORCE,
                                        false);

  GripperCtrl ctrl_msg;
  ctrl_msg.target_width = gripper_ctrl_position;
  ctrl_msg.target_force = gripper_ctrl_effort;
  ctrl_msg.control = 1;
  return ctrl_msg;
}

void OnRobotRG2FTActionServer::publishJointStates(const GripperState::SharedPtr gripper_state)
{
  double position_radians = mapRange(
      gripper_state->actual_gripper_width,
      MIN_POSITION, MAX_POSITION,
      gripper_params_.min_angle_, gripper_params_.max_angle_, true);

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = {gripper_params_.joint_name_};
  msg.position = {position_radians};

  joint_states_pub_->publish(msg);
}

double OnRobotRG2FTActionServer::mapRange(double val, double prev_min, double prev_max,
                                          double new_min, double new_max, bool reverse)
{
  double ret = (val - prev_min) * (new_max - new_min) / (prev_max - prev_min) + new_min;
  return reverse ? new_max - ret : ret;
}

}  // namespace onrobot_rg2ft_action_server

