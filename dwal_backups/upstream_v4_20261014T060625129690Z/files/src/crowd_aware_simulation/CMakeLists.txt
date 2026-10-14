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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(dynamic_dwal_node src/dynamic_dwal_node.cpp)
target_include_directories(dynamic_dwal_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dynamic_dwal_node PUBLIC cxx_std_17)
ament_target_dependencies(dynamic_dwal_node
  cohan_msgs crowd_aware_interfaces geometry_msgs nav_msgs rclcpp
  sensor_msgs visualization_msgs)

# Original fixed sources; no new geometry core.
add_library(dwal_minimal_lib STATIC src/libdwal_cluster.cpp)
target_include_directories(dwal_minimal_lib BEFORE PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/dwal_minimal>)
target_compile_features(dwal_minimal_lib PUBLIC cxx_std_17)
ament_target_dependencies(dwal_minimal_lib
  dwal_planner geometry_msgs nav_msgs nav2_costmap_2d rclcpp tf2 tf2_geometry_msgs)

add_executable(dwal_generator_fixed src/trajectory_generator.cpp)
add_executable(dwal_clustering_fixed src/clustering.cpp)
foreach(target dwal_generator_fixed dwal_clustering_fixed)
  target_include_directories(${target} BEFORE PRIVATE include/dwal_minimal)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_link_libraries(${target} dwal_minimal_lib)
  ament_target_dependencies(${target}
    cohan_msgs dwal_planner geometry_msgs nav_msgs nav2_costmap_2d
    rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)
endforeach()

install(TARGETS dynamic_dwal_node dwal_generator_fixed dwal_clustering_fixed dwal_minimal_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
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
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dynamic_dwal_core test/test_dynamic_dwal_core.cpp)
  if(TARGET test_dynamic_dwal_core)
    target_include_directories(test_dynamic_dwal_core PRIVATE include)
    target_compile_features(test_dynamic_dwal_core PUBLIC cxx_std_17)
  endif()
endif()

ament_export_include_directories(include)
ament_package()
