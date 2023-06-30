cmake_minimum_required(VERSION 3.8)
project(rpl_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# custom functions
include(cmake/register_executable.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)

# rclcpp
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# messages
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)

# rpl
find_package(rpl REQUIRED)
find_package(rpl_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
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

include_directories(include)

register_executable(world_descriptor
  "src/world_descriptor.cpp"
  "rclcpp;rclcpp_components;obstacles_msgs;geometry_msgs;rpl;rpl_msgs"
)

install(TARGETS
  world_descriptor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
