#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_tutorials_interfaces/action/move_robot.hpp"

class NavigationServer : public rclcpp::Node
{
public:
  using MoveRobot = action_tutorials_interfaces::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

  NavigationServer() : Node("navigation_server")
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<MoveRobot>(
      this,
      "navigate_robot",
      std::bind(&NavigationServer::handle_goal, this, _1, _2),
      std::bind(&NavigationServer::handle_cancel, this, _1),
      std::bind(&NavigationServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Server activated, waiting for goals...");
  }

private:
  rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveRobot::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Goal received! Go to X: %.2f, Y: %.2f, theta: %.2f", goal->target_x, goal->target_y, goal->target_theta);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Cancellation accepted.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  
  void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
  {
    
    auto result = std::make_shared<MoveRobot::Result>();
    result->final_x = 0.0; 
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal completed!");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationServer>());
  rclcpp::shutdown();
  return 0;
}