#include "wp_swarm.hh"

void gazebo::WP_Swarm::OnUpdate(const common::UpdateInfo &_info)
{
	std::lock_guard<std::mutex> lock(this->mutex);
	// std::cerr << _info.simTime.Double() << " " << _info.realTime.Double() <<std::endl;
}