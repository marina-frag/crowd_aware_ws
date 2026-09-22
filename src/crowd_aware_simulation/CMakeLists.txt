cmake_minimum_required(VERSION 3.8)
project(crowd_aware_simulation)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch rviz scripts config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/hunav_to_cohan_bridge.py
  scripts/cohan_agents_info_adapter.py
  scripts/context_estimator.py
  scripts/dynamic_radius_policy.py
  scripts/command_guard.py
  scripts/final_cmd_watchdog.py
  scripts/local_task.py
  scripts/teleop_path_adapter.py
  scripts/policy_core.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
