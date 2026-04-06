cmake_minimum_required(VERSION 3.8)
project(assem)

# Compiler options for GNUCXX and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament_cmake package
find_package(ament_cmake REQUIRED)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install non-code resources (launch, urdf, meshes, rviz, textures, config)
install(DIRECTORY
  launch
  urdf
  meshes
  rviz
  textures
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()