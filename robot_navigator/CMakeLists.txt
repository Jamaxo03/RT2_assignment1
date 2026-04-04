cmake_minimum_required(VERSION 3.8)
project(robot_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(nav_server_base src/navigation_server.cpp)
ament_target_dependencies(nav_server_base rclcpp rclcpp_action action_tutorials_interfaces geometry_msgs nav_msgs)

add_executable(nav_client_base src/navigation_client.cpp)
ament_target_dependencies(nav_client_base rclcpp rclcpp_action action_tutorials_interfaces)

install(TARGETS
  nav_server_base
  nav_client_base
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
