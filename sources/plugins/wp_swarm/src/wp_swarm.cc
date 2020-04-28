#include "wp_swarm.hh"

void gazebo::WP_Swarm::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{
	this->node = transport::NodePtr(new transport::Node());
	this->node->Init();
	this->world = _parent;
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&WP_Swarm::OnUpdate,this,_1));
}

gazebo::WorldPlugin* gazebo::RegisterPlugin()
{
return new gazebo::WP_Swarm();
}