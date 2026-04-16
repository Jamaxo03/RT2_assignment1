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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

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

add_library(nav_server_component SHARED src/navigation_server.cpp)
ament_target_dependencies(nav_server_component 
  rclcpp rclcpp_action action_tutorials_interfaces rclcpp_components 
  geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
rclcpp_components_register_node(nav_server_component 
  PLUGIN "robot_navigator::NavigationServer" 
  EXECUTABLE nav_server_node)

add_library(nav_client_component SHARED src/navigation_client.cpp)
ament_target_dependencies(nav_client_component 
  rclcpp rclcpp_action action_tutorials_interfaces rclcpp_components)
rclcpp_components_register_node(nav_client_component 
  PLUGIN "robot_navigator::NavigationClient" 
  EXECUTABLE nav_client_node)

install(TARGETS
  nav_server_component
  nav_client_component
  EXPORT export_${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME}_components)

ament_package()
