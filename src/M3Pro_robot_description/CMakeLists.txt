cmake_minimum_required(VERSION 3.5)

project(M3Pro_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes urdf rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
