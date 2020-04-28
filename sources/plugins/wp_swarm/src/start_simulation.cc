#include <start_simulation.hh>

void cb_simulation_status(ConstAnyPtr &world_status)
{//check if simulation has started
    std::lock_guard<std::mutex> lock(mutex);
    simulation_started = world_status->bool_value();
}

int main(int _argc, char **_argv)
{
    std::string paramsfile{""};
	int paramsline{0};

    gazebo::msgs::Any params_msg;
    params_msg.set_type(gazebo::msgs::Any::STRING);
    params_msg.set_string_value(std::to_string(paramsline)
             + ":" + paramsfile);

    //load gazebo
    gazebo::client::setup(_argc, _argv);

    //create node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

    //listen for simulation status
    gazebo::transport::SubscriberPtr sub_simulation_status;
	sub_simulation_status = node->Subscribe("/simulation_status",cb_simulation_status);
	
    //publisher for simulation parameters
    gazebo::transport::PublisherPtr pub_simulation_params = node->Advertise<gazebo::msgs::Any>("/simulation_params");

	while(!simulation_started){//busy wait
		gazebo::common::Time::MSleep(100);
		pub_simulation_params->Publish(params_msg);
	}
	
	//Make sure to shut everything down
	gazebo::client::shutdown();
}