cmake_minimum_required(VERSION 3.8)
project(crowd_aware_simulation)
find_package(ament_cmake REQUIRED)

# DWAL C++ executables are built by the patched upstream dwal_planner package.
install(DIRECTORY launch rviz scripts config scenarios worlds
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/hunav_to_cohan_bridge.py
  scripts/record_dwal.py
  scripts/plot_dwal_bag.py
  scripts/evaluate_dwal_bag.py
  scripts/replay_reference.py
  scripts/dwal_runtime_check.py
  scripts/reference_task.py
  scripts/constant_speed_limit.py
  scripts/command_guard.py
  scripts/final_cmd_watchdog.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_robot_geometry
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_geometry.py)
endif()

# SSOT configuration helpers
# Helpers are loaded via their share path, not as ROS executables.
install(FILES scripts/setup_config.py scripts/configured_tools.py
  DESTINATION share/${PROJECT_NAME}/scripts)
# Include nested controllers/ and runtime.json as well as the existing YAML files.
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# Local imports must work both from share/scripts and ros2 run's lib directory.
install(FILES scripts/evaluation_config.py scripts/dwal_bag.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
