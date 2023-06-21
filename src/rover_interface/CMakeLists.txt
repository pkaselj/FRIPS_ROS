cmake_minimum_required(VERSION 3.8)
project(rover_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_definitions(-DASIO_STANDALONE)
add_compile_definitions(ASIO_STANDALONE)

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  set (CMAKE_CXX_FLAGS "-g -ggdb")
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_messages REQUIRED)
find_package(rover_interface_definition REQUIRED)
#find_package(asio REQUIRED)

add_library(stxetx_parse_fsm include/${PROJECT_NAME}/fsm.hpp src/fsm.cpp)
target_include_directories(stxetx_parse_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  stxetx_parse_fsm
  "rclcpp"
)

add_executable(rover_interface_node src/rover_interface_node.cpp)

if(WIN32)
  target_include_directories(rover_interface_node PUBLIC "A:\\asio-1.28.0\\include")
endif()
#target_link_libraries(rover_interface_node asio)
target_include_directories(rover_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(rover_interface_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(rover_interface_node stxetx_parse_fsm)
ament_target_dependencies(
  rover_interface_node
  "rclcpp"
  "std_msgs"
  "rover_messages"
  "rover_interface_definition"
)

install(TARGETS rover_interface_node
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
