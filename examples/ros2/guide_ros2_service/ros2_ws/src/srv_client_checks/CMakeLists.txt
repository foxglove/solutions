cmake_minimum_required(VERSION 3.8)
project(srv_client_checks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# add nodes as executables with their dependencies
add_executable(motor_node src/motor_node.cpp)
ament_target_dependencies(motor_node 
  rclcpp std_srvs)
add_executable(diagnostics_node src/diagnostics_node.cpp)
ament_target_dependencies(diagnostics_node
  rclcpp std_srvs)

# install so ros2 run can find them 
install(TARGETS
  motor_node
  diagnostics_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
