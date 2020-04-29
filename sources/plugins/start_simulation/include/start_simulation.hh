#ifndef START_SIMULATION_HH
#define START_SIMULATION_HH

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "gazebo/common/common.hh"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <mutex>

#include "simulation_params.pb.h"

std::mutex mutex;

//callback function to end client
void cb_simulation_status(ConstAnyPtr &sim_status);
bool simulation_started{false};

#endif