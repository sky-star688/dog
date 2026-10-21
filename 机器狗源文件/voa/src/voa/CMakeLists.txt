cmake_minimum_required(VERSION 3.5)
project(voa)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(cv_bridge REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(grid_map_loader REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_octomap REQUIRED)
find_package(grid_map_rviz_plugin REQUIRED)
find_package(grid_map_visualization REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(gridmap_dep
    grid_map_core
    grid_map_ros
    grid_map_cv
    grid_map_filters
    grid_map_loader
    grid_map_msgs
    grid_map_octomap
    grid_map_rviz_plugin
    grid_map_visualization
)

set(msg_dep
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
)

# add_definitions(-DDEBUG_GRIDMAP="")
# add_definitions(-DDEBUG_MOTION="")
# add_definitions(-DDEBUG_TF="")
# add_definitions(-DDEBUG_MARKER="")

add_executable(voa_composition src/VoaComposition.cpp)
ament_target_dependencies(voa_composition rclcpp ${gridmap_dep} ${msg_dep} pcl_ros cv_bridge OpenCV)

add_executable(fake_imu_publisher src/FakeImuPublisher.cpp)
ament_target_dependencies(fake_imu_publisher rclcpp ${msg_dep})

set(targets_list
    voa_composition
    fake_imu_publisher
)
install(
    TARGETS ${targets_list}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY rviz launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${gridmap_dep} ${msg_dep})
ament_package()
