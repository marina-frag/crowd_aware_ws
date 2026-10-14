cmake_minimum_required(VERSION 3.8)
project(crowd_aware_simulation)
find_package(ament_cmake REQUIRED)

# DWAL C++ executables are built by the patched upstream dwal_planner package.
install(DIRECTORY launch rviz scripts config scenarios worlds
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/hunav_to_cohan_bridge.py
  scripts/cohan_agents_info_adapter.py
  scripts/constant_speed_limit.py
  scripts/context_speed_limit.py
  scripts/context_estimator.py
  scripts/command_guard.py
  scripts/final_cmd_watchdog.py
  scripts/local_task.py
  scripts/teleop_path_adapter.py
  scripts/smoke_check.py
  scripts/dynamic_dwal_check.py
  scripts/policy_core.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_robot_geometry
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_geometry.py)
endif()
ament_package()
