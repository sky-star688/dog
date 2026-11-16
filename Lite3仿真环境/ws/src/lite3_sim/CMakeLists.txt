cmake_minimum_required(VERSION 3.5)
project(lite3_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/stand_pose.py
  scripts/arm_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
