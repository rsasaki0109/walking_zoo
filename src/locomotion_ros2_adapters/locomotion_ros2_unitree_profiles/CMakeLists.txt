cmake_minimum_required(VERSION 3.8)
project(locomotion_ros2_unitree_profiles)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
