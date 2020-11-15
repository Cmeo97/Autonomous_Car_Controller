cmake_minimum_required(VERSION 2.8.3)
project(pcl_solution)


find_package(catkin REQUIRED COMPONENTS
	pcl_conversions
	pcl_ros
  	roscpp
	vision_msgs
  	sensor_msgs
	geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs geometry_msgs vision_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_solution_node src/pcl_solution_node.cpp)
target_link_libraries(pcl_solution_node ${catkin_LIBRARIES})
