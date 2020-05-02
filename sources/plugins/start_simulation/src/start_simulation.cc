#include <start_simulation.hh>

void cb_simulation_status(ConstAnyPtr &world_status)
{//check if simulation has started
    std::lock_guard<std::mutex> lock(mutex);
    simulation_started = world_status->bool_value();
}

int main(int _argc, char **_argv)
{
    std::string simulation_folder{_argv[1]};
    std::string params_file{_argv[2]};
	std::string params_line{_argv[3]};
    std::string simulation_prefix{_argv[4]};
    
    custom_msgs::msgs::SimulationParams params_msg;
    params_msg.set_simulation_folder(simulation_folder);
    params_msg.set_simulation_prefix(simulation_prefix);
    params_msg.set_params("Default");
    params_msg.set_params_file(params_file);
    params_msg.set_params_line(std::stoi(params_line));

    //load gazebo
    gazebo::client::setup(_argc, _argv);

    //create node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

    //listen for simulation status
    gazebo::transport::SubscriberPtr sub_simulation_status;
	sub_simulation_status = node->Subscribe("/simulation_status",cb_simulation_status);
	
    //publisher for simulation parameters
    gazebo::transport::PublisherPtr pub_simulation_params = node->Advertise<custom_msgs::msgs::SimulationParams>("/simulation_params");

	while(!simulation_started){//busy wait
		gazebo::common::Time::MSleep(100);
		pub_simulation_params->Publish(params_msg);
        // std::cerr << "simulation_folder: " << simulation_folder << std::endl
        // << "params_file: " << params_file <<std::endl
        // << "params_line: " << params_line << std::endl
        // << "simulation_prefix: " << simulation_prefix << std::endl;
	}
	
	//Make sure to shut everything down
	gazebo::client::shutdown();
}