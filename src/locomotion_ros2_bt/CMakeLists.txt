cmake_minimum_required(VERSION 3.8)
project(locomotion_ros2_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)
find_package(locomotion_ros2_msgs REQUIRED)

set(dependencies
  ament_index_cpp
  behaviortree_cpp
  rclcpp
  locomotion_ros2_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/check_walking_ready.cpp
  src/clear_walking_fault.cpp
  src/locomotion_ros2_bt_nodes.cpp
  src/recovery_bt.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
# Export the BT_RegisterNodesFromPlugin entry point so the library can also be
# loaded dynamically via BT::BehaviorTreeFactory::registerFromPlugin(...).
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)

add_executable(locomotion_ros2_bt_recovery_node src/recovery_node.cpp)
target_compile_features(locomotion_ros2_bt_recovery_node PUBLIC cxx_std_17)
target_link_libraries(locomotion_ros2_bt_recovery_node ${PROJECT_NAME})
ament_target_dependencies(locomotion_ros2_bt_recovery_node ${dependencies})

# Nav2-loadable BT plugin library: IsWalkingReady + ClearWalkingFault, following
# the Nav2 bt_navigator plugin convention (node from blackboard, BtServiceNode).
# Kept in its own library so the core locomotion_ros2_bt stays Nav2-free; only this
# target pulls in nav2_behavior_tree, and it is what goes in `plugin_lib_names`.
add_library(locomotion_ros2_nav2_bt_nodes SHARED src/nav2_recovery_nodes.cpp)
target_compile_features(locomotion_ros2_nav2_bt_nodes PUBLIC cxx_std_17)
target_include_directories(locomotion_ros2_nav2_bt_nodes PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(locomotion_ros2_nav2_bt_nodes ${PROJECT_NAME})
ament_target_dependencies(locomotion_ros2_nav2_bt_nodes ${dependencies} nav2_behavior_tree)
target_compile_definitions(locomotion_ros2_nav2_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

# Harness that ticks the recovery branch through the real Nav2 BT engine.
add_executable(locomotion_ros2_nav2_recovery_harness src/nav2_recovery_harness.cpp)
target_compile_features(locomotion_ros2_nav2_recovery_harness PUBLIC cxx_std_17)
ament_target_dependencies(locomotion_ros2_nav2_recovery_harness
  ament_index_cpp behaviortree_cpp rclcpp nav2_behavior_tree)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY bt_xml launch config DESTINATION share/${PROJECT_NAME})
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME} locomotion_ros2_nav2_bt_nodes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS locomotion_ros2_bt_recovery_node locomotion_ros2_nav2_recovery_harness
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_locomotion_ros2_bt_nodes test/test_locomotion_ros2_bt_nodes.cpp)
  if(TARGET test_locomotion_ros2_bt_nodes)
    target_link_libraries(test_locomotion_ros2_bt_nodes ${PROJECT_NAME})
    ament_target_dependencies(test_locomotion_ros2_bt_nodes behaviortree_cpp locomotion_ros2_msgs)
    target_compile_definitions(test_locomotion_ros2_bt_nodes PRIVATE
      "LOCOMOTION_ROS2_BT_XML_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/bt_xml/locomotion_ros2_nav2_recovery.xml\"")
  endif()

  ament_add_gtest(test_recovery_bt test/test_recovery_bt.cpp)
  if(TARGET test_recovery_bt)
    target_link_libraries(test_recovery_bt ${PROJECT_NAME})
    ament_target_dependencies(test_recovery_bt ${dependencies})
    target_compile_definitions(test_recovery_bt PRIVATE
      "LOCOMOTION_ROS2_RECOVERY_XML_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/bt_xml/locomotion_ros2_recovery_live.xml\"")
  endif()

  # Loads the Nav2 plugin library the same way bt_navigator does and drives the
  # IsWalkingReady / ClearWalkingFault nodes against a fake runtime.
  ament_add_gtest(test_nav2_bt_recovery_nodes test/test_nav2_bt_recovery_nodes.cpp)
  if(TARGET test_nav2_bt_recovery_nodes)
    add_dependencies(test_nav2_bt_recovery_nodes locomotion_ros2_nav2_bt_nodes)
    ament_target_dependencies(test_nav2_bt_recovery_nodes
      behaviortree_cpp rclcpp locomotion_ros2_msgs nav2_behavior_tree)
    target_compile_definitions(test_nav2_bt_recovery_nodes PRIVATE
      "LOCOMOTION_ROS2_NAV2_BT_NODES_LIB=\"$<TARGET_FILE:locomotion_ros2_nav2_bt_nodes>\"")
  endif()
endif()

ament_package()
