cmake_minimum_required(VERSION 3.8)
project(amr_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(velocity_controller src/velocity_controller.cpp)
add_executable(Sensor_test src/Sensor_test.cpp)

ament_target_dependencies(velocity_controller 
  rclcpp 
  geometry_msgs 
  sensor_msgs
)

ament_target_dependencies(Sensor_test
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
)

install(
  TARGETS velocity_controller Sensor_test
  DESTINATION lib/${PROJECT_NAME}
)

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
