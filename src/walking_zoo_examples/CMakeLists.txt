cmake_minimum_required(VERSION 3.8)
project(walking_zoo_examples)

find_package(ament_cmake REQUIRED)

install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/mujoco_g1_gait_demo.py
    scripts/send_estop.py
    scripts/send_mock_cmd_vel.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
