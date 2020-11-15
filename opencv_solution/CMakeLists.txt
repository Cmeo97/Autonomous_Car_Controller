cmake_minimum_required(VERSION 3.0.2)
project(opencv_solution)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp prius_msgs cv_bridge image_transport message_generation sensor_msgs std_msgs vision_msgs)
find_package(OpenCV REQUIRED)
# Declare our catkin package.
catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(${PROJECT_NAME}_node src/opencv_solution_node.cpp)

# Specify libraries against which to link.
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#target_link_libraries(simple_control ${prius_msgs_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})
