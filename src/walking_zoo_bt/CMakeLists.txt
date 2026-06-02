cmake_minimum_required(VERSION 3.8)
project(walking_zoo_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(walking_zoo_msgs REQUIRED)

set(dependencies
  ament_index_cpp
  behaviortree_cpp
  rclcpp
  walking_zoo_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/check_walking_ready.cpp
  src/clear_walking_fault.cpp
  src/walking_zoo_bt_nodes.cpp
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

add_executable(walking_zoo_bt_recovery_node src/recovery_node.cpp)
target_compile_features(walking_zoo_bt_recovery_node PUBLIC cxx_std_17)
target_link_libraries(walking_zoo_bt_recovery_node ${PROJECT_NAME})
ament_target_dependencies(walking_zoo_bt_recovery_node ${dependencies})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY bt_xml launch DESTINATION share/${PROJECT_NAME})
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS walking_zoo_bt_recovery_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_walking_zoo_bt_nodes test/test_walking_zoo_bt_nodes.cpp)
  if(TARGET test_walking_zoo_bt_nodes)
    target_link_libraries(test_walking_zoo_bt_nodes ${PROJECT_NAME})
    ament_target_dependencies(test_walking_zoo_bt_nodes behaviortree_cpp walking_zoo_msgs)
    target_compile_definitions(test_walking_zoo_bt_nodes PRIVATE
      "WALKING_ZOO_BT_XML_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/bt_xml/walking_zoo_nav2_recovery.xml\"")
  endif()

  ament_add_gtest(test_recovery_bt test/test_recovery_bt.cpp)
  if(TARGET test_recovery_bt)
    target_link_libraries(test_recovery_bt ${PROJECT_NAME})
    ament_target_dependencies(test_recovery_bt ${dependencies})
    target_compile_definitions(test_recovery_bt PRIVATE
      "WALKING_ZOO_RECOVERY_XML_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/bt_xml/walking_zoo_recovery_live.xml\"")
  endif()
endif()

ament_package()
