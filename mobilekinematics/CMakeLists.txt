cmake_minimum_required(VERSION 3.5)
project(mobilekinematics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add any other dependencies here using find_package

ament_package()


install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/pid_params.py
    src/mecanum_bot.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  DIRECTORY
    description
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



# Add any other install directives here
