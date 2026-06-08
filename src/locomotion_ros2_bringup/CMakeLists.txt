cmake_minimum_required(VERSION 3.8)
project(locomotion_ros2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params rviz maps config DESTINATION share/${PROJECT_NAME})

ament_package()
