cmake_minimum_required(VERSION 3.5)
project(turtlebot3_on_robot_stack)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
