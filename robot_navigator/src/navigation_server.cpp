#include <memory>
#include <thread>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_tutorials_interfaces/action/move_robot.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

class NavigationServer : public rclcpp::Node
{
public:
  using MoveRobot = action_tutorials_interfaces::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

  NavigationServer() : Node("navigation_server")
  {
    using namespace std::placeholders;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&NavigationServer::odom_callback, this, _1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<GoalHandleMoveRobot> active_goal_handle_;

  std::mutex control_mutex_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "base_world"; 
    t.child_frame_id = "base_footprint"; 

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    t.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const MoveRobot::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(control_mutex_);
    
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
       RCLCPP_INFO(this->get_logger(), "goal_frame overwrited!");
    }
    
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
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      active_goal_handle_ = goal_handle;
    }
    std::thread{std::bind(&NavigationServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
  {
    rclcpp::Rate loop_rate(10); 
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveRobot::Result>();
    auto twist_msg = geometry_msgs::msg::Twist();

    while (rclcpp::ok()) {

      {
        std::lock_guard<std::mutex> lock(control_mutex_);
        if (goal_handle != active_goal_handle_) {
          goal_handle->abort(result);
          return; 
        }
      }
      
      if (goal_handle->is_canceling()) {
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(twist_msg);
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled!");
        return;
      }

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();

      t.header.frame_id = "base_world"; 
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

      geometry_msgs::msg::TransformStamped chase_tf;
      try {
        chase_tf = tf_buffer_->lookupTransform("base_footprint", "goal_frame", tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        loop_rate.sleep();
        continue;
      }

      // first reach goal position
      double err_x = chase_tf.transform.translation.x;
      double err_y = chase_tf.transform.translation.y;

      double distance = std::sqrt(std::pow(err_x, 2) + std::pow(err_y, 2));
      double angle_to_goal = std::atan2(err_y, err_x);

      if (distance > 0.1) {
        
        twist_msg.angular.z = 1.5 * angle_to_goal;
        twist_msg.linear.x = 0.5 * distance;
    
        if (twist_msg.linear.x > 0.5) {
           twist_msg.linear.x = 0.5;
        }

      } 
      // after reaching position, adjust orientation
      else {
        tf2::Quaternion qu(chase_tf.transform.rotation.x,chase_tf.transform.rotation.y,chase_tf.transform.rotation.z,chase_tf.transform.rotation.w);

        tf2::Matrix3x3 m(qu);
        double roll, pitch, yaw_error;
        m.getRPY(roll, pitch, yaw_error);
        if (std::abs(yaw_error) > 0.05) { 
          twist_msg.linear.x = 0.0;    
          twist_msg.angular.z = 1.0 * yaw_error;
        } else {
          break; 
        }
      }

      cmd_vel_pub_->publish(twist_msg);
      loop_rate.sleep();
    }

    // STOP
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(twist_msg);

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Reached Successfully!");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationServer>());
  rclcpp::shutdown();
  return 0;
}