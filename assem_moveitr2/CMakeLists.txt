cmake_minimum_required(VERSION 3.22)
project(assem_moveitr2)

find_package(ament_cmake REQUIRED)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE
  )
else()
  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()