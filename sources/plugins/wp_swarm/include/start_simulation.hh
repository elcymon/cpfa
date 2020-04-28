/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef START_SIMULATION_HH
#define START_SIMULATION_HH

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "gazebo/common/common.hh"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <mutex>


std::mutex mutex;

//callback function to end client
void cb_simulation_status(ConstAnyPtr &sim_status);
bool simulation_started{false};

#endif