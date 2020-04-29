# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/custom_msgs
  )
link_directories(${GAZEBO_LIBRARY_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/custom_msgs)

# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(ALLOW_DUPLICATE_CUSTOM_TARGETS TRUE)

#if(NOT TARGET custom_msgs)
    add_subdirectory(custom_msgs)
#endif(NOT TARGET custom_msgs)

# header files directory
include_directories(include)

# start simulations gazebo client
set (START_SIMULATION src/start_simulation.cc
    include/start_simulation.hh)
add_executable(start_simulation ${START_SIMULATION})
target_link_libraries(start_simulation
    custom_msgs
    ${GAZEBO_LIBRARIES} pthread
    ${Boost_LIBRARIES} ${SDF_LIBRARIES})

# flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")