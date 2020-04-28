cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# gazebo related files
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# header files directory
include_directories(include)

# world plugin for robots simulation
set(WP_SWARM src/wp_swarm.cc
    src/wp_swarm_onupdate.cc
    include/wp_swarm.hh)
add_library(wp_swarm SHARED ${WP_SWARM})
target_link_libraries(wp_swarm ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# start simulations gazebo client
set (START_SIMULATION src/start_simulation.cc
    include/start_simulation.hh)
add_executable(start_simulation  ${START_SIMULATION})
target_link_libraries(start_simulation ${GAZEBO_LIBRARIES} pthread)

# flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")