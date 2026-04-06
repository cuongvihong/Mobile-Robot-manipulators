cmake_minimum_required(VERSION 3.8)
project(myrobot_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

# =========================
# Include directories
# =========================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# =========================
# IK library
# =========================
add_library(myrobot_ik
  src/ik_myrobot.cpp
)

target_include_directories(myrobot_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(myrobot_ik
  rclcpp
  moveit_ros_planning_interface
)

# =========================
# test_moveit executable
# =========================
add_executable(test_moveit
  src/test_moveit.cpp
)

target_link_libraries(test_moveit
  myrobot_ik
)

ament_target_dependencies(test_moveit
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
  moveit_msgs
)

# =========================
# test_moveitr2 executable
# =========================
add_executable(test_moveitr2
  src/test_moveitr2.cpp
)

target_link_libraries(test_moveitr2
  myrobot_ik
)

ament_target_dependencies(test_moveitr2
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
  moveit_msgs
)

# =========================
# test_moveitr3 executable
# =========================
add_executable(test_moveitr3
  src/test_moveitr3.cpp
)

target_link_libraries(test_moveitr3
  myrobot_ik
)

ament_target_dependencies(test_moveitr3
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
  moveit_msgs
)

# =========================
# waypoint_terminal executable
# =========================
add_executable(waypoint_terminal
  src/waypoint_terminal.cpp
)

ament_target_dependencies(waypoint_terminal
  rclcpp
  std_msgs
)

# =========================
# arm_waypoint_terminal executable
# =========================
add_executable(arm_waypoint_terminal
  src/arm_waypoint_terminal.cpp
)

ament_target_dependencies(arm_waypoint_terminal
  rclcpp
  std_msgs
  geometry_msgs
)

# =========================
# cartesianpath executable
# =========================
add_executable(cartesianpath
  src/cartesianpath.cpp
)

ament_target_dependencies(cartesianpath
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

# =========================
# Install headers
# =========================
install(
  DIRECTORY include/
  DESTINATION include
)

# =========================
# Install executables and libs
# =========================
install(
  TARGETS
    myrobot_ik
    test_moveit
    test_moveitr2
    test_moveitr3
    waypoint_terminal
    arm_waypoint_terminal
    cartesianpath
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Install launch files
# =========================
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# Package
# =========================
ament_package()