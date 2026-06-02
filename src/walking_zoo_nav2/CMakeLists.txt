cmake_minimum_required(VERSION 3.8)
project(walking_zoo_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(walking_zoo_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/cmd_vel_bridge.cpp
  src/legged_velocity_shaper.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME} geometry_msgs rclcpp walking_zoo_msgs)

add_executable(cmd_vel_bridge src/cmd_vel_bridge_main.cpp)
target_compile_features(cmd_vel_bridge PUBLIC cxx_std_17)
target_link_libraries(cmd_vel_bridge ${PROJECT_NAME})
ament_target_dependencies(cmd_vel_bridge geometry_msgs rclcpp walking_zoo_msgs)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME} cmd_vel_bridge
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs rclcpp walking_zoo_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_legged_velocity_shaper test/test_legged_velocity_shaper.cpp)
  if(TARGET test_legged_velocity_shaper)
    target_link_libraries(test_legged_velocity_shaper ${PROJECT_NAME})
  endif()
endif()

ament_package()
