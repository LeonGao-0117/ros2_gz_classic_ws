cmake_minimum_required(VERSION 3.8)
project(M3Pro_robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/multi_lidar_merger.py
    scripts/collect_waypoints.py
    scripts/send_goal_to_waypoint.py
    scripts/reset_to_init_pose.py
    scripts/waypoint_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/utils
  DESTINATION lib/${PROJECT_NAME}/scripts
)

ament_package()
