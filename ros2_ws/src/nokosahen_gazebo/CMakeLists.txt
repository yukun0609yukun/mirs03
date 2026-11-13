cmake_minimum_required(VERSION 3.8)
project(nokosahen_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  worlds
  launch
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
