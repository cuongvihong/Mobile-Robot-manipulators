cmake_minimum_required(VERSION 3.8)
project(myrobot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  PROGRAMS
    scripts/tf_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
