cmake_minimum_required(VERSION 3.8)
project(aubo_description)
find_package(ament_cmake REQUIRED)
# The upstream mesh and URDF directories are read-only workspace links.
install(DIRECTORY mesh/ DESTINATION share/${PROJECT_NAME}/mesh)
install(DIRECTORY mesh/ DESTINATION share/${PROJECT_NAME}/meshes/aubo_i5)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY xacro DESTINATION share/${PROJECT_NAME})
ament_package()
