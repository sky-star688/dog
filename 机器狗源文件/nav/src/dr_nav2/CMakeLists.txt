cmake_minimum_required(VERSION 3.15)
project(dr_nav2)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_auto_package()
