cmake_minimum_required(VERSION 3.8)
project(walking_zoo_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params rviz maps DESTINATION share/${PROJECT_NAME})

ament_package()
