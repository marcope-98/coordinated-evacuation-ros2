cmake_minimum_required(VERSION 3.8)
project(simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# rclcpp
find_package(rclcpp REQUIRED)

# messages
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)

# other
find_package(tf2_ros REQUIRED)

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

# nodes executable
add_executable(send_gates src/send_gates.cpp src/utils.hpp)
add_executable(send_borders src/send_borders.cpp src/utils.hpp)
add_executable(send_obstacles src/send_obstacles.cpp src/utils.hpp)
add_executable(get_positions src/get_positions.cpp)

# nodes dependies
ament_target_dependencies(send_gates
  rclcpp
  std_msgs
  gazebo_msgs
  geometry_msgs
)

ament_target_dependencies(send_borders
  rclcpp
  std_msgs
  geometry_msgs
)

ament_target_dependencies(get_positions
  rclcpp
  geometry_msgs
  tf2_ros
)

ament_target_dependencies(send_obstacles
  rclcpp
  std_msgs
  gazebo_msgs
  geometry_msgs
  obstacles_msgs
)

# install
install(TARGETS
  send_gates
  send_borders
  send_obstacles
  get_positions
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  worlds
  models
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
