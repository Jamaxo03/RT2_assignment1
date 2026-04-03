#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_tutorials_interfaces/action/move_robot.hpp"

class NavigationClient : public rclcpp::Node
{
public:
  using MoveRobot = action_tutorials_interfaces::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MoveRobot>;

  NavigationClient() : Node("navigation_client")
  {
    client_ptr_ = rclcpp_action::create_client<MoveRobot>(this, "navigate_robot");
  }

  void send_goal()
  {
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(60))) {
      RCLCPP_ERROR(this->get_logger(), "Server not available.");
      return;
    }

    auto goal_msg = MoveRobot::Goal();
    goal_msg.target_x = 1.0; 
    goal_msg.target_y = 1.0; 
    goal_msg.target_theta = 1.0; 

    RCLCPP_INFO(this->get_logger(), "Sending goal to server...");

    auto send_goal_options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavigationClient::goal_response_callback, this, std::placeholders::_1);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<MoveRobot>::SharedPtr client_ptr_;

  void goal_response_callback(const GoalHandleMoveRobot::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal rejected by server.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server!");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<NavigationClient>();
  
  client_node->send_goal();
  
  rclcpp::spin(client_node);
  rclcpp::shutdown();
  return 0;
}