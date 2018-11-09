cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

#add_message_files(
#        FILES
#)

#add_service_files( DIRECTORY srv
#        FILES
#        )

generate_messages(DEPENDENCIES std_msgs sensor_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES openpose_ros
   CATKIN_DEPENDS roscpp rospy message_runtime std_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories( include )
include_directories(
  ${catkin_INCLUDE_DIRS}
)


#MESSAGE(OpenCV_VERSION: ${OpenCV_VERSION})
#MESSAGE(OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS})

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
