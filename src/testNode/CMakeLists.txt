cmake_minimum_required(VERSION 2.8.3)
project(first)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()
message("$PROJECT_SOURCE_DIR")
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker ${PROJECT_SOURCE_DIR}/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener ${PROJECT_SOURCE_DIR}/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
