cmake_minimum_required(VERSION 3.0.2)
project(door_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  gazebo_ros
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs pcl_ros pcl_conversions gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(edge_detection_node src/edge_detection_node.cpp)
target_link_libraries(edge_detection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

