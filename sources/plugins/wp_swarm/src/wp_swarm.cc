#include "wp_swarm.hh"

void gazebo::WP_Swarm::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{
	this->node = transport::NodePtr(new transport::Node());
	this->node->Init();
	this->world = _parent;
	this->sub_simulation_params = this->node->Subscribe("/simulation_params",&WP_Swarm::cb_simulation_params,this);
	this->simulation_params = std::make_shared<custom_msgs::msgs::SimulationParams>();
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&WP_Swarm::OnUpdate,this,_1));
	
}
void gazebo::WP_Swarm::cb_simulation_params(gazebo::ConstSimulationParamsPtr &params_msg)
{
	std::lock_guard<std::mutex> lock(this->mutex);	
	std::	fstream params_file(params_msg->params_file());
	params_file.seekg(std::ios::beg);
	std::string params_header,params;
	std::vector<std::string> params_vec,params_header_vec;
	params_file >> params_header;
	boost::split(params_header_vec,
		params_header,boost::is_any_of(","));
	
	for (int i = 0; i < params_msg->params_line(); i++)
	{//go to specific line in params file
		params_file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
	}
	params_file >> params;
	boost::split(params_vec,params,boost::is_any_of(","));
	for (int  i = 0; i < (int) params_vec.size(); i++)
	{
		this->params_map[params_header_vec[i]] = params_vec[i];
		std::cerr << params_header_vec[i] << " : " << params_vec[i] << std::endl;
	}
	this->simulation_params->set_params(params_msg->params());
	this->simulation_params->set_simulation_folder(params_msg->simulation_folder());
	this->simulation_params->set_simulation_prefix(params_msg->simulation_prefix());
	this->simulation_params->set_params_file(params_msg->params_file());
	this->simulation_params->set_params_line(params_msg->params_line());
	this->sim_results_prefix = params_msg->simulation_folder() + "/" + 
			this->params_map["ID"] + params_msg->simulation_prefix();
	std::cerr << params_header << std::endl;
	std::cerr << params << std::endl;
	std::cerr <<std::endl <<this->sim_results_prefix <<std::endl;
	exit(0);
}
gazebo::WorldPlugin* gazebo::RegisterPlugin()
{
return new gazebo::WP_Swarm();
}