cmake_minimum_required(VERSION 3.8)
project(rover_pose_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rover_messages REQUIRED)
find_package(marvelmind_ros2_msgs REQUIRED)

add_executable(rover_pose_estimator_node src/rover_pose_estimator_node.cpp)
target_include_directories(rover_pose_estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rover_pose_estimator_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  rover_pose_estimator_node
  "rclcpp"
  "geometry_msgs"
  "turtlesim"
  "rover_messages"
  "marvelmind_ros2_msgs"
)

install(TARGETS rover_pose_estimator_node
  DESTINATION lib/${PROJECT_NAME})

# DEPRECATED, all configuration is managed by rover_launcher package
# install(
#     DIRECTORY
#     config
#     DESTINATION
#     share/${PROJECT_NAME}
#   )

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
