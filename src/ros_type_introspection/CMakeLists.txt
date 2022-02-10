cmake_minimum_required(VERSION 2.8.3)
project(ros_type_introspection)
find_package(catkin REQUIRED COMPONENTS
   #roscpp
   rostime 
   roscpp_serialization
   #abseil_cpp
)
# Build flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -g -std=c++11 -fPIC")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ros_type_introspection
   CATKIN_DEPENDS 
   roscpp 
   rostime 
   roscpp_serialization
   #abseil_cpp
   DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${HISI_TOP}/toolchain/profiler_ext/include
)
link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ library
 add_library(ros_type_introspection
   src/ros_type.cpp
   src/ros_field.cpp
   src/ros_message.cpp
   src/ros_introspection.cpp
 )

target_link_libraries(ros_type_introspection ${catkin_LIBRARIES} roscpp)


#############
## Install ##
#############

install(TARGETS ros_type_introspection 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".git" EXCLUDE
)

#############
## Testing ##
#############

