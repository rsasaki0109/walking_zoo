cmake_minimum_required(VERSION 3.8)
project(walking_zoo_vla)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(walking_zoo_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/semantic_action_mapper.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME} geometry_msgs walking_zoo_msgs)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs walking_zoo_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_semantic_action_mapper test/test_semantic_action_mapper.cpp)
  if(TARGET test_semantic_action_mapper)
    target_link_libraries(test_semantic_action_mapper ${PROJECT_NAME})
  endif()
endif()

ament_package()
