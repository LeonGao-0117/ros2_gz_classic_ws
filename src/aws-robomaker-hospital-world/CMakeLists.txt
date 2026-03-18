cmake_minimum_required(VERSION 3.5)
project(aws_robomaker_hospital_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models fuel_models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()