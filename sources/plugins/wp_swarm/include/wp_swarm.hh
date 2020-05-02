#ifndef WP_SWARM_HH
#define WP_SWARM_HH

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <string>
#include <fstream>
#include <limits>
#include <vector>
#include <map>

#include "simulation_params.pb.h"

using namespace std;
namespace gazebo
{
	typedef const boost::shared_ptr<
  const custom_msgs::msgs::SimulationParams>
    ConstSimulationParamsPtr;
	class WP_Swarm : public WorldPlugin
	{
		//Pointer to the update event connection
		private:
			event::ConnectionPtr updateConnection;
			physics::WorldPtr world;
			std::mutex mutex;
			transport::NodePtr node;
			std::string sim_results_prefix;
			std::map<std::string,std::string> params_map;

			std::shared_ptr<custom_msgs::msgs::SimulationParams> simulation_params;
			gazebo::transport::SubscriberPtr sub_simulation_params;
		public:
			void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
			void OnUpdate(const common::UpdateInfo &_info);
			void cb_simulation_params(ConstSimulationParamsPtr &params_msg);
			
	};
	
	//Register this plugin with the simulator
	//GZ_REGISTER_WORLD_PLUGIN(WP_Swarm)
    extern "C" gazebo::WorldPlugin *RegisterPlugin();
}

#endif