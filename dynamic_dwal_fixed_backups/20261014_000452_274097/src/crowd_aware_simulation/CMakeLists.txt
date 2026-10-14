cmake_minimum_required(VERSION 3.8)
project(crowd_aware_simulation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cohan_msgs REQUIRED)
find_package(crowd_aware_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(dwal_planner REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(dynamic_dwal_node src/dynamic_dwal_node.cpp)
target_include_directories(dynamic_dwal_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dynamic_dwal_node PUBLIC cxx_std_17)
ament_target_dependencies(dynamic_dwal_node
  cohan_msgs crowd_aware_interfaces geometry_msgs nav_msgs rclcpp
  sensor_msgs visualization_msgs)

add_library(dynamic_dwal_fixed_core src/dynamic_dwal_fixed_core.cpp)
target_include_directories(dynamic_dwal_fixed_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dynamic_dwal_fixed_core PUBLIC cxx_std_17)

add_executable(dynamic_dwal_fixed src/dynamic_dwal_fixed_node.cpp)
target_compile_features(dynamic_dwal_fixed PUBLIC cxx_std_17)
target_link_libraries(dynamic_dwal_fixed dynamic_dwal_fixed_core)
ament_target_dependencies(dynamic_dwal_fixed
  cohan_msgs crowd_aware_interfaces dwal_planner geometry_msgs nav_msgs
  nav2_costmap_2d rclcpp sensor_msgs std_msgs visualization_msgs)

install(TARGETS dynamic_dwal_node dynamic_dwal_fixed dynamic_dwal_fixed_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch rviz scripts config scenarios worlds
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/wait_dynamic_dwal_fixed.py
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
  add_executable(test_dynamic_dwal_fixed_core test/test_dynamic_dwal_fixed_core.cpp)
  target_link_libraries(test_dynamic_dwal_fixed_core dynamic_dwal_fixed_core)
  target_compile_features(test_dynamic_dwal_fixed_core PUBLIC cxx_std_17)
  add_test(NAME test_dynamic_dwal_fixed_core COMMAND test_dynamic_dwal_fixed_core)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_robot_geometry
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_geometry.py)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dynamic_dwal_core test/test_dynamic_dwal_core.cpp)
  if(TARGET test_dynamic_dwal_core)
    target_include_directories(test_dynamic_dwal_core PRIVATE include)
    target_compile_features(test_dynamic_dwal_core PUBLIC cxx_std_17)
  endif()
endif()

ament_export_include_directories(include)
ament_package()
