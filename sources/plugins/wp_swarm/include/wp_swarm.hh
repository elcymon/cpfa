#ifndef WP_SWARM_HH
#define WP_SWARM_HH

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <boost/bind.hpp>

#include <fstream>
#include <limits>

#include "simulation_params.pb.h"

using namespace std;
namespace gazebo
{
	class WP_Swarm : public WorldPlugin
	{
		//Pointer to the update event connection
		private:
			event::ConnectionPtr updateConnection;
			physics::WorldPtr world;
			std::mutex mutex;
			transport::NodePtr node;
		public:
			void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
			void OnUpdate(const common::UpdateInfo &_info);
	};
	
	//Register this plugin with the simulator
	//GZ_REGISTER_WORLD_PLUGIN(WP_Swarm)
    extern "C" gazebo::WorldPlugin *RegisterPlugin();
}

#endif