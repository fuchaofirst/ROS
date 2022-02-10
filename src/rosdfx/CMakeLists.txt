cmake_minimum_required(VERSION 2.8.3)
project(rosdfx)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_type_introspection   #fuchao  
  )

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
set(CMAKE_CXX_FLAGS "-g -std=c++11 ${CMAKE_CXX_FLAGS}") 
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
catkin_package_xml()

include_directories(include ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros ${catkin_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
 # INCLUDE_DIRS include ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros
  #LIBRARIES roscpp 
 # )
message("################"${PROJECT_SOURCE_DIR})
include_directories(include ${PROJECT_SOURCE_DIR}/rosdfx/include)


link_directories(
  ${CMAKE_INSTALL_PREFIX}/lib
  /opt/ros/melodic/lib
  )
#node 

add_executable(rostopic src/tools/rostopic.cpp ) 


target_link_libraries(rostopic ${catkin_LIBRARIES} roscpp  ros_type_introspection topic_tools)


install(TARGETS rostopic
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")



