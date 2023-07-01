cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME rpl)
set(LIBRARY_NAME rpl)

set(LIBRARY_INCLUDE_DIR include/${LIBRARY_NAME})
set(LIBRARY_SRC_DIR src)
set(LIBRARY_SRC

  # clipper
  ${LIBRARY_SRC_DIR}/clipper/clipper.cpp

  # io
  ${LIBRARY_SRC_DIR}/io/WorldDescriptor.cpp
  ${LIBRARY_SRC_DIR}/io/Minkowski.cpp

  # map
  ${LIBRARY_SRC_DIR}/map/AVLTree.cpp
  ${LIBRARY_SRC_DIR}/map/Graph.cpp
  ${LIBRARY_SRC_DIR}/map/RoadMap.cpp
)

project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2 -march=native)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

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

add_library(${LIBRARY_NAME}
  SHARED
  ${LIBRARY_SRC})

target_include_directories(${LIBRARY_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}>
  $<INSTALL_INTERFACE:include/${LIBRARY_NAME}>)

target_link_libraries(${LIBRARY_NAME}
  PUBLIC
  PRIVATE)

ament_export_targets(${LIBRARY_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY ${LIBRARY_INCLUDE_DIR}
  DESTINATION include)

install(
  TARGETS ${LIBRARY_NAME}
  EXPORT ${LIBRARY_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_package()
