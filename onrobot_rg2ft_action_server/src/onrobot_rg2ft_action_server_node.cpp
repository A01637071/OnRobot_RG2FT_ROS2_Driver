#include "onrobot_rg2ft_action_server/onrobot_rg2ft_action_server.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("onrobot_rg2ft_action_server");

  onrobot_rg2ft_action_server::GripperParams cparams;

  // Declarar parámetros
  node->declare_parameter<std::string>("action_server_name", "gripper_controller_gripper_cmd");
  node->declare_parameter<double>("min_angle", 0.0);
  node->declare_parameter<double>("max_angle", 0.93);
  node->declare_parameter<double>("min_effort", 3.0);
  node->declare_parameter<double>("max_effort", 40.0);
  node->declare_parameter<double>("default_effort", 10.0);
  node->declare_parameter<std::string>("control_topic", "gripper/ctrl");
  node->declare_parameter<std::string>("state_topic", "gripper/states");
  node->declare_parameter<std::string>("joint_states_topic", "joint_states");
  node->declare_parameter<std::string>("joint_name", "left_outer_knuckle_joint");

  // Cargar parámetros
  std::string action_server_name;
  node->get_parameter("action_server_name", action_server_name);
  node->get_parameter("min_angle", cparams.min_angle_);
  node->get_parameter("max_angle", cparams.max_angle_);
  node->get_parameter("min_effort", cparams.min_effort_);
  node->get_parameter("max_effort", cparams.max_effort_);
  node->get_parameter("default_effort", cparams.default_effort_);
  node->get_parameter("control_topic", cparams.control_topic_);
  node->get_parameter("state_topic", cparams.state_topic_);
  node->get_parameter("joint_states_topic", cparams.joint_states_topic_);
  node->get_parameter("joint_name", cparams.joint_name_);

  RCLCPP_INFO(node->get_logger(), "Initializing OnRobot RG2-FT Gripper action server: %s",
              action_server_name.c_str());

  // Crear instancia del servidor (sin action_server_name como argumento)
  auto gripper_node = std::make_shared<onrobot_rg2ft_action_server::OnRobotRG2FTActionServer>(
    "onrobot_rg2ft_action_server",
    cparams,
    action_server_name,
    rclcpp::NodeOptions());


  RCLCPP_INFO(node->get_logger(),
              "Action server spinning for OnRobot RG2-FT Gripper: %s", action_server_name.c_str());

  rclcpp::spin(gripper_node);
  rclcpp::shutdown();
  return 0;
}

