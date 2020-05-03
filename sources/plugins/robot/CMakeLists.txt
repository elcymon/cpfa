# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  include
  )
link_directories(${GAZEBO_LIBRARY_DIRS})

FILE(GLOB ROBOT src/*.cc include/*.hh)

# add_library(wp_swarm SHARED ${WP_SWARM})
# target_link_libraries(wp_swarm
#    custom_msgs
#    ${GAZEBO_LIBRARIES} pthread
#    ${Boost_LIBRARIES} ${SDF_LIBRARIES})
