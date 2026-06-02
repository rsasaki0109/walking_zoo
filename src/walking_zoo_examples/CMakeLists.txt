cmake_minimum_required(VERSION 3.8)
project(walking_zoo_examples)

find_package(ament_cmake REQUIRED)

install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/mujoco_g1_gait_demo.py
    scripts/mujoco_g1_gait_showcase.py
    scripts/send_estop.py
    scripts/send_mock_cmd_vel.py
    scripts/walking_zoo_demo_recorder.py
    scripts/walking_zoo_lerobot_export.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_lerobot_export test/test_lerobot_export.py)
  # Skips itself if HuggingFace 'datasets'/'pyarrow' are unavailable.
  ament_add_pytest_test(test_lerobot_hf_compat test/test_lerobot_hf_compat.py)
endif()

ament_package()
