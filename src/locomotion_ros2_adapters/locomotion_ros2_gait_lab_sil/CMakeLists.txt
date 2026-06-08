cmake_minimum_required(VERSION 3.8)
project(locomotion_ros2_gait_lab_sil)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
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

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES plugins.xml README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs pluginlib rclcpp std_msgs locomotion_ros2_core locomotion_ros2_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gait_lab_sil_model test/test_gait_lab_sil_model.cpp)
  if(TARGET test_gait_lab_sil_model)
    target_link_libraries(test_gait_lab_sil_model ${PROJECT_NAME})
  endif()
endif()

ament_package()
