cmake_minimum_required(VERSION 3.8)
project(locomotion_ros2_unitree_sdk2)

option(LOCOMOTION_ROS2_WITH_UNITREE_SDK2 "Build with vendor Unitree SDK2 support" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(locomotion_ros2_core REQUIRED)
find_package(locomotion_ros2_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/unitree_loco_command.cpp
  src/sil_loco_backend.cpp
  src/sdk2_loco_backend.cpp
  src/unitree_sdk2_adapter.cpp
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
  locomotion_ros2_core
  locomotion_ros2_msgs
)

if(LOCOMOTION_ROS2_WITH_UNITREE_SDK2)
  # Link the vendor Unitree SDK2. Point CMake at the SDK install with either
  # -Dunitree_sdk2_DIR=<sdk>/lib/cmake/unitree_sdk2 or by adding the SDK prefix to
  # CMAKE_PREFIX_PATH. find_package fails loudly when it is missing, so an ON
  # build never silently degrades to the software-in-the-loop backend.
  find_package(unitree_sdk2 REQUIRED)
  target_compile_definitions(${PROJECT_NAME} PRIVATE LOCOMOTION_ROS2_WITH_UNITREE_SDK2=1)
  target_link_libraries(${PROJECT_NAME} unitree_sdk2)
  message(STATUS "locomotion_ros2_unitree_sdk2: building with vendor Unitree SDK2 support")
else()
  message(STATUS
    "locomotion_ros2_unitree_sdk2: software-in-the-loop build; configure with "
    "-DLOCOMOTION_ROS2_WITH_UNITREE_SDK2=ON to link the vendor SDK")
endif()

pluginlib_export_plugin_description_file(locomotion_ros2_core plugins.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES plugins.xml README.md DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs pluginlib rclcpp locomotion_ros2_core locomotion_ros2_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_unitree_loco_command test/test_unitree_loco_command.cpp)
  if(TARGET test_unitree_loco_command)
    target_link_libraries(test_unitree_loco_command ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_loco_backend test/test_loco_backend.cpp)
  if(TARGET test_loco_backend)
    target_link_libraries(test_loco_backend ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_unitree_sdk2_adapter test/test_unitree_sdk2_adapter.cpp)
  if(TARGET test_unitree_sdk2_adapter)
    target_link_libraries(test_unitree_sdk2_adapter ${PROJECT_NAME})
  endif()
endif()

ament_package()
