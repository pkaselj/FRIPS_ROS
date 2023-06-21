cmake_minimum_required(VERSION 3.8)
project(rover_interface_definition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# let the compiler search for headers in the include folder
include_directories(include)
 
add_library(rover_interface_definition src/stxetx_protocol.c)
 
ament_export_targets(rover_interface_definition HAS_LIBRARY_TARGET)
 
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)


install(
  TARGETS rover_interface_definition
  EXPORT rover_interface_definition
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
