#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_tutorials_interfaces/action/move_robot.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class NavigationServer : public rclcpp::Node
{
public:
  using MoveRobot = action_tutorials_interfaces::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

  NavigationServer() : Node("navigation_server")
  {
    using namespace std::placeholders;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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
    RCLCPP_INFO(this->get_logger(), "Cancellation accepted");
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
    auto result = std::make_shared<MoveRobot::Result>();

    while (rclcpp::ok()) {
      
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled!");
        return;
      }

      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "base_footprint"; 
      t.child_frame_id = "goal_frame";     

      t.transform.translation.x = goal->target_x;
      t.transform.translation.y = goal->target_y;
      t.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, goal->target_theta);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(t);

      loop_rate.sleep();
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