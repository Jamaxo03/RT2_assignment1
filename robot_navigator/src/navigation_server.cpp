#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_tutorials_interfaces/action/move_robot.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class NavigationServer : public rclcpp::Node
{
public:
  using MoveRobot = action_tutorials_interfaces::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

  NavigationServer() : Node("navigation_server")
  {
    using namespace std::placeholders;

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&NavigationServer::odom_callback, this, _1));

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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  double current_x_ = 0.0; 

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
  }

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
    std::thread{std::bind(&NavigationServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
  {
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveRobot::Feedback>();
    auto result = std::make_shared<MoveRobot::Result>();
    auto twist_msg = geometry_msgs::msg::Twist();

    while (rclcpp::ok() && current_x_ < goal->target_x) {
      
      if (goal_handle->is_canceling()) {
        twist_msg.linear.x = 0.0;
        cmd_vel_pub_->publish(twist_msg); 
        result->final_x = current_x_;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled!");
        return;
      }

      twist_msg.linear.x = 0.2;
      cmd_vel_pub_->publish(twist_msg);

      feedback->current_x = current_x_;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      twist_msg.linear.x = 0.0;
      cmd_vel_pub_->publish(twist_msg); 
      
      result->final_x = current_x_;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal completed! Reached X: %.2f", current_x_);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationServer>());
  rclcpp::shutdown();
  return 0;
}