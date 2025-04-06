cmake_minimum_required(VERSION 3.8)
project(amr_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# 添加可执行文件
add_executable(map_publisher src/map_publisher.cpp)

# 设置 ROS 2 依赖
ament_target_dependencies(map_publisher   
  rclcpp
  nav_msgs
  ament_index_cpp
  tf2_ros
  geometry_msgs)

# 链接非 ROS 2 库（OpenCV 和 yaml-cpp）
target_link_libraries(map_publisher 
  ${OpenCV_LIBRARIES} 
  yaml-cpp 
  tf2_ros::tf2_ros
  tf2::tf2
  )

# 安装目标
install(TARGETS map_publisher DESTINATION lib/${PROJECT_NAME})

# 安装额外的资源文件（launch 和 map 文件）
install(DIRECTORY launch map DESTINATION share/${PROJECT_NAME})

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
