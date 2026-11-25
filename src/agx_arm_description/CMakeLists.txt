cmake_minimum_required(VERSION 3.16)
project(agx_arm_description)

find_package(ament_cmake REQUIRED)

# URDF/xacro 与 mesh
install(DIRECTORY agx_arm_urdf meshes
  DESTINATION share/${PROJECT_NAME})

ament_package()
