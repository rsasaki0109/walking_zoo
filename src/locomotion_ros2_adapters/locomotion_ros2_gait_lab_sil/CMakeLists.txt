cmake_minimum_required(VERSION 3.8)
project(locomotion_ros2_gait_lab_sil)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(locomotion_ros2_core REQUIRED)
find_package(locomotion_ros2_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/gait_lab_sil_model.cpp
  src/gait_lab_sil_adapter.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  pluginlib
  rclcpp
  std_msgs
  locomotion_ros2_core
  locomotion_ros2_msgs
)

pluginlib_export_plugin_description_file(locomotion_ros2_core plugins.xml)

add_library(${PROJECT_NAME}_ros2_control SHARED
  src/gait_lab_sil_topic_system.cpp
)
target_compile_features(${PROJECT_NAME}_ros2_control PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_ros2_control PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME}_ros2_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)
pluginlib_export_plugin_description_file(hardware_interface plugins_ros2_control.xml)

add_library(${PROJECT_NAME}_ros2_control_controllers SHARED
  src/gait_lab_sil_joint_forward_controller.cpp
  src/gait_lab_sil_rl_residual_controller.cpp
  src/gait_lab_rl_policy.cpp
)
target_compile_features(${PROJECT_NAME}_ros2_control_controllers PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_ros2_control_controllers PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME}_ros2_control_controllers
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)
target_link_libraries(${PROJECT_NAME}_ros2_control_controllers
  controller_interface::controller_interface
)
pluginlib_export_plugin_description_file(
  controller_interface plugins_ros2_control_controllers.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  FILES plugins.xml plugins_ros2_control.xml plugins_ros2_control_controllers.xml README.md
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_ros2_control ${PROJECT_NAME}_ros2_control_controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface geometry_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle
  sensor_msgs std_msgs
  locomotion_ros2_core locomotion_ros2_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gait_lab_sil_model test/test_gait_lab_sil_model.cpp)
  if(TARGET test_gait_lab_sil_model)
    target_link_libraries(test_gait_lab_sil_model ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_gait_lab_rl_policy test/test_gait_lab_rl_policy.cpp)
  if(TARGET test_gait_lab_rl_policy)
    target_link_libraries(test_gait_lab_rl_policy ${PROJECT_NAME}_ros2_control_controllers)
  endif()
endif()

ament_package()
