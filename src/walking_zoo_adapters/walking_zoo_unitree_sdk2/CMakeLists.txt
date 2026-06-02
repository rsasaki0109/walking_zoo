cmake_minimum_required(VERSION 3.8)
project(walking_zoo_unitree_sdk2)

option(WALKING_ZOO_WITH_UNITREE_SDK2 "Build with vendor Unitree SDK2 support" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(walking_zoo_core REQUIRED)
find_package(walking_zoo_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/unitree_loco_command.cpp
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
  walking_zoo_core
  walking_zoo_msgs
)

if(WALKING_ZOO_WITH_UNITREE_SDK2)
  target_compile_definitions(${PROJECT_NAME} PRIVATE WALKING_ZOO_WITH_UNITREE_SDK2=1)
endif()

pluginlib_export_plugin_description_file(walking_zoo_core plugins.xml)

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
ament_export_dependencies(geometry_msgs pluginlib rclcpp walking_zoo_core walking_zoo_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_unitree_loco_command test/test_unitree_loco_command.cpp)
  if(TARGET test_unitree_loco_command)
    target_link_libraries(test_unitree_loco_command ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_unitree_sdk2_adapter test/test_unitree_sdk2_adapter.cpp)
  if(TARGET test_unitree_sdk2_adapter)
    target_link_libraries(test_unitree_sdk2_adapter ${PROJECT_NAME})
  endif()
endif()

ament_package()
