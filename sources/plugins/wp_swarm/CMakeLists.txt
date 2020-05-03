# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/custom_msgs
  robot/src robot/include
  include
  )
link_directories(${GAZEBO_LIBRARY_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/custom_msgs)

add_subdirectory(custom_msgs)

FILE(GLOB WP_SWARM src/*.cc include/*.hh
        robot/src/*.cc robot/include/*.hh)

add_library(wp_swarm SHARED ${WP_SWARM} )
target_link_libraries(wp_swarm
    custom_msgs
    ${GAZEBO_LIBRARIES} pthread
    ${Boost_LIBRARIES} ${SDF_LIBRARIES})

# flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")