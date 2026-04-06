cmake_minimum_required(VERSION 3.8)
project(mobilerobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------
# Find packages
# -------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# -------------------------
# Install Python nodes
# -------------------------
install(PROGRAMS
  src/team_task_manager_v2.py
  src/team_task_manager_v3.py
  src/team_task_manager_v4.py
  src/team_task_manager_v5.py

  src/team_status_manager.py
  src/team_status_manager_v2.py

  src/payload_state_estimator.py
  src/payload_planner_v2.py
  src/pickup_place_coordinator.py
  src/formation_generator.py

  src/route_segmenter.py
  src/local_path_follower.py
  src/mecanum_pose_tracker.py
  src/cmd_vel_to_mecanum_ref.py

  src/arm_task_executor.py
  src/object_attach_manager.py
  src/object_pose_to_payload_pose.py
  src/team_safety_monitor_v2.py

  src/graph_builder_from_iris.py
  src/graph_builder_from_map.py
  src/graph_builder_from_waypoints.py



  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------
# Install launch/config folders
# -------------------------
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------
# ament package
# -------------------------
ament_package()