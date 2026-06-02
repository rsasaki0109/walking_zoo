cmake_minimum_required(VERSION 3.8)
project(walking_zoo_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(walking_zoo_core REQUIRED)
find_package(walking_zoo_msgs REQUIRED)
find_package(walking_zoo_safety REQUIRED)

set(dependencies
  geometry_msgs
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  visualization_msgs
  walking_zoo_core
  walking_zoo_msgs
  walking_zoo_safety
)

add_library(${PROJECT_NAME}
  src/adapter_loader.cpp
  src/command_arbiter.cpp
  src/footstep_planner.cpp
  src/step_feasibility_checker.cpp
  src/mode_manager.cpp
  src/walking_runtime_manager.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(walking_zoo_runtime_manager src/runtime_main.cpp)
target_compile_features(walking_zoo_runtime_manager PUBLIC cxx_std_17)
target_link_libraries(walking_zoo_runtime_manager ${PROJECT_NAME})
ament_target_dependencies(walking_zoo_runtime_manager ${dependencies})

add_executable(footstep_marker_publisher src/footstep_marker_node.cpp)
target_compile_features(footstep_marker_publisher PUBLIC cxx_std_17)
target_link_libraries(footstep_marker_publisher ${PROJECT_NAME})
ament_target_dependencies(footstep_marker_publisher ${dependencies})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME} walking_zoo_runtime_manager footstep_marker_publisher
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_arbiter test/test_command_arbiter.cpp)
  if(TARGET test_command_arbiter)
    target_link_libraries(test_command_arbiter ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_mode_manager test/test_mode_manager.cpp)
  if(TARGET test_mode_manager)
    target_link_libraries(test_mode_manager ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_footstep_planner test/test_footstep_planner.cpp)
  if(TARGET test_footstep_planner)
    target_link_libraries(test_footstep_planner ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_step_feasibility_checker test/test_step_feasibility_checker.cpp)
  if(TARGET test_step_feasibility_checker)
    target_link_libraries(test_step_feasibility_checker ${PROJECT_NAME})
  endif()
endif()

ament_package()
