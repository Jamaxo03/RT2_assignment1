#include <memory>
#include <thread>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_tutorials_interfaces/action/move_robot.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace robot_navigator
{
  class NavigationClient : public rclcpp::Node
  {
  public:
    using MoveRobot = action_tutorials_interfaces::action::MoveRobot;
    using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MoveRobot>;

    explicit NavigationClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
    : Node("navigation_client", options)
    {
      client_ptr_ = rclcpp_action::create_client<MoveRobot>(this, "navigate_robot");
      ui_thread_ = std::thread(&NavigationClient::ui_loop, this);
    }

    ~NavigationClient() {
      if (ui_thread_.joinable()) {
        ui_thread_.join();
      }
    }

  private:
    rclcpp_action::Client<MoveRobot>::SharedPtr client_ptr_;
    std::thread ui_thread_;
    GoalHandleMoveRobot::SharedPtr current_goal_handle_;

    void ui_loop()
    {
      while (!client_ptr_->wait_for_action_server(std::chrono::seconds(2))) {
        if (!rclcpp::ok()) return;
        RCLCPP_INFO(this->get_logger(), "Waiting for Action Server...");
      }

      while (rclcpp::ok()) {
        std::cout << "\n========== MENU ==========\n";
        std::cout << "1. Set a new Target (x, y, theta)\n";
        std::cout << "2. Cancel current Target\n";
        std::cout << "Choose an option: ";
        
        int choice;
        std::cin >> choice;

        if (choice == 1) {
          float x, y, theta;
          std::cout << "Enter target X: "; std::cin >> x;
          std::cout << "Enter target Y: "; std::cin >> y;
          std::cout << "Enter target Theta: "; std::cin >> theta;
          send_goal(x, y, theta);
        } 
        else if (choice == 2) {
          cancel_goal();
        } 
        else {
          std::cout << "Invalid choice. Try again.\n";
        }
      }
    }

    void send_goal(float x, float y, float theta)
    {
      auto goal_msg = MoveRobot::Goal();
      goal_msg.target_x = x; 
      goal_msg.target_y = y; 
      goal_msg.target_theta = theta; 

      RCLCPP_INFO(this->get_logger(), "Sending goal to server...");

      auto send_goal_options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
      send_goal_options.goal_response_callback =
        std::bind(&NavigationClient::goal_response_callback, this, std::placeholders::_1);

      send_goal_options.result_callback =
        std::bind(&NavigationClient::result_callback, this, std::placeholders::_1);

      client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void cancel_goal()
    {
      if (current_goal_handle_) {
        RCLCPP_INFO(this->get_logger(), "Sending cancel request...");
        client_ptr_->async_cancel_goal(current_goal_handle_);
      } else {
        RCLCPP_WARN(this->get_logger(), "No active goal to cancel!");
      }
    }

    void goal_response_callback(const GoalHandleMoveRobot::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal rejected by server.");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server!");
        current_goal_handle_ = goal_handle;
      }
    }

    void result_callback(const GoalHandleMoveRobot::WrappedResult & result)
    {
      if (current_goal_handle_ && current_goal_handle_->get_goal_id() == result.goal_id) {
        current_goal_handle_ = nullptr; 
      }

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Goal Succeeded!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal Aborted by server.");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal successfully Canceled!");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
          break;
      }
    }
  };
} 

RCLCPP_COMPONENTS_REGISTER_NODE(robot_navigator::NavigationClient)