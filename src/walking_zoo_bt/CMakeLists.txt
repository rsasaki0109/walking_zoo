cmake_minimum_required(VERSION 3.8)
project(walking_zoo_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(walking_zoo_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/check_walking_ready.cpp
  src/clear_walking_fault.cpp
  src/walking_zoo_bt_nodes.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME} behaviortree_cpp walking_zoo_msgs)
# Export the BT_RegisterNodesFromPlugin entry point so the library can also be
# loaded dynamically via BT::BehaviorTreeFactory::registerFromPlugin(...).
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY bt_xml DESTINATION share/${PROJECT_NAME})
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp walking_zoo_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_walking_zoo_bt_nodes test/test_walking_zoo_bt_nodes.cpp)
  if(TARGET test_walking_zoo_bt_nodes)
    target_link_libraries(test_walking_zoo_bt_nodes ${PROJECT_NAME})
    ament_target_dependencies(test_walking_zoo_bt_nodes behaviortree_cpp walking_zoo_msgs)
    target_compile_definitions(test_walking_zoo_bt_nodes PRIVATE
      "WALKING_ZOO_BT_XML_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/bt_xml/walking_zoo_nav2_recovery.xml\"")
  endif()
endif()

ament_package()
