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

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "gazebo/common/common.hh"
#include <gazebo/gazebo.hh>

#include <gazebo/physics/physics.hh>

#include <iostream>
#include <fstream>
#include <set>
#include <mutex>

#include <string>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <time.h>
#include <sys/stat.h>
#include <errno.h>

#include <vector>

std::mutex mutex1;
//#include <gazebo/Server.hh>

using namespace std;



std::string folder_name = "";//initialize directory to save results
std::string start_time = "";
std::string end_time = "";
std::string prefix = "";
std::string prefix1,prefix2;
std::string sim_data; //used to store simulation parameters
std::string algorithmID = "";
int algorithm_index = 0;
int paramLine = 0;//index entered by the user from terminal
double sim_time = 1000000;
//bool prefix_set = false;
bool start_sim;

bool world_loaded = false;

bool world_gov_control_bool = false;//initialized to false to prevent premature start of the simulation

time_t message_time = time(nullptr);

auto tt = std::time(nullptr);//used to prefix start and end time. Should be generated based on world_gov_experiment_control
auto ttm = *std::localtime(&tt);

void litter_pose_cb(ConstAnyPtr &any)
{std::lock_guard<std::mutex> lock(mutex1);
	//if(start_sim)
	{
		std::string lit_pos = any->string_value();
		//ofstream myfile(folder_name + prefix + "litter_pose.txt",std::ios::app|std::ios::ate);
		//myfile << lit_pos<<std::endl;
		//myfile.close();
	}
}

void topic_model_log_cb(ConstAnyPtr &any)
{std::lock_guard<std::mutex> lock(mutex1);
		time(&message_time);
	
		std::string log = any->string_value();
		
		std::size_t pos = log.find(":");
		std::string robot_name = log.substr(0,pos);
		ofstream myfile(folder_name + prefix +"_"+ robot_name+".txt",std::ios::app|std::ios::ate);
		myfile << log.substr(pos+1) << std::endl;
		myfile.close();
	
}
void litter_in_nest_cb(ConstAnyPtr &any)
{
	std::lock_guard<std::mutex> lock(mutex1);
	time(&message_time);
	//if(start_sim)
	{
		std::string lit_count = any->string_value();
		std::size_t pos = lit_count.find(",");
		std::string time_string = lit_count.substr(0,pos);
		// double curr_time = std::stod(time_string);

		// sim_time = curr_time;
		ofstream myfile2(folder_name + prefix + "_litter_count.txt",std::ios::app|std::ios::ate);
		myfile2 <<lit_count<<std::endl;
		myfile2.close();
	}
}
void start_sim_cb(ConstAnyPtr &any)
{//detect when a new experiment starts and name file appropriately
	std::lock_guard<std::mutex> lock(mutex1);
	start_sim = any->bool_value();
	if(!start_sim)
	{//when world plugin wants experiments to stop set world control to false
		// world_gov_control_bool = false;
		// ostringstream si1;
		// auto t = std::time(nullptr);
		// auto tm = *std::localtime(&t);
		// si1 << std::put_time(&tm,"%Y-%m-%d--%H-%M-%S_");
		// //prefix = si1.str();
		// std::cout<<prefix<<std::endl;
		
	}
}
void sub_world_loaded_cb(ConstAnyPtr &any){
	//check for when world has been loaded
	std::lock_guard<std::mutex> lock(mutex1);
	world_loaded = any->bool_value();
}

void experiment_control_cb(ConstAnyPtr &any)
{//create a readme.md file for each folder where you put info about each experiment in folder
	std::lock_guard<std::mutex> lock(mutex1);
	std::string data = any->string_value();
	
	
	
	tt = std::time(nullptr);	
	ofstream myfile2(folder_name + "readme.md",std::ios::app|std::ios::ate);
	if(data.compare("end") !=0)
	{
		ttm = *std::localtime(&tt);
		ostringstream si1;
		si1 <<algorithm_index<<"-"<<algorithmID<< std::put_time(&ttm,"-%Y-%m-%d--%H-%M-%S");
		prefix = si1.str();
		si1.str("");
		si1.clear();
		si1 << data << std::put_time(&ttm,",start_time:%Y-%m-%d--%H-%M-%S");
		sim_data = si1.str();//data ;
		
		// myfile2 //<<std::put_time(&ttm,"end_time:%Y-%m-%d--%H-%M-%S\n")
		// 		<<data
		// 		<<std::put_time(&ttm,",start_time:%Y-%m-%d--%H-%M-%S,");
	}
	if(data.substr(0,3).compare("end")==0 and world_gov_control_bool)
	{
		world_gov_control_bool = false;
		ttm = *std::localtime(&tt);
		myfile2 <<"prefix:"<<prefix<<","<<sim_data
				<<std::put_time(&ttm,",end_time:%Y-%m-%d--%H-%M-%S\n");
	}
	myfile2.close();
}
/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
	//Read params.csv file:
	//All params to be simulated are stored in a vector
	vector<string> my_params;
	std::vector<string>::iterator param_it;
	
	ifstream myfile;//for loading file
	string line, header;//which row in file and row 1 is header
	int param_it_loc = 0;//keep track of which location in my_params vector we are at
	myfile.open("params/params.csv");
	if(myfile.is_open()){
		getline(myfile,header);
		paramLine = std::stoi(_argv[2]);
		int paramCounter = 0;
		while(getline(myfile,line)){
			paramCounter++;
			if(paramCounter == paramLine){
				size_t hsep1 = 0, hsep2 = 0, psep1 = 0, psep2 = 0;
				string hparam, param_value;
				string param_line = "";
				while(header.find(",",hsep1) != string::npos and
						line.find(",",psep1) != string::npos)
				{
					hsep2 = header.find(",",hsep1);
					hparam = header.substr(hsep1,hsep2-hsep1);
					psep2 = line.find(",",psep1);
					param_value = line.substr(psep1,psep2-psep1);
					
					param_line = param_line + hparam + ":" + param_value + ",";
					hsep1 = hsep2 + 1;
					psep1 = psep2 + 1;

					if(hparam.compare("ID") == 0){
						algorithmID = param_value;
					}
				}
				my_params.push_back(param_line);
				if(paramLine > 0){//if param line is 0, read everything if it is greater than 0 seek desired line
					break;//exit loop if parameter line is found.
				}
				else if(paramLine < 0){
					exit(5);

				}
			}
		}
		//iterator set to first parameter list
		param_it = my_params.begin();
		if (paramLine > 0){
			param_it_loc = 0;
		}
		else{
			param_it_loc = 1;
		}
		
		myfile.close();
	}
	else
	{
		//cout<<"unable to load file"<<endl;
		exit(5);
	}


	//set up folder name to save results
	ostringstream si1;
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	si1 << std::put_time(&tm,"%Y-%m-%d-");
	
	folder_name =  "./results/" + si1.str()+ _argv[1] + "/";
	const char *mk_dir = folder_name.c_str();
	
	int a = mkdir("results", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (a==0 or errno==EEXIST)
	{
		if(mkdir(mk_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
		{
			if(errno==EEXIST){
				//already exists
				std::cout<<folder_name<<" already exists"<<endl;
			}
			else {
				std::cout<<"cannot create: "<<folder_name<<" errno: "<<errno<<endl;
				exit(errno);
			}
		}
		// ofstream myfilex(folder_name + "readme.md",std::ios::app|std::ios::ate);
		// myfilex <<"\n**************\n"
		// 		  <<"New Experiment"
		// 		<<"\n**************\n";
		// myfilex.close();
	}
	else
	{
		std::cout<<"something went wrong while creating results folder. Errno = "<<errno<<endl;
		exit(errno);
	}
		
	
	
	
	//Load gazebo
	gazebo::client::setup(_argc, _argv);
	
	//create our node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	
	//listen to when world is loaded
	gazebo::transport::SubscriberPtr sub_litter_pose;
	sub_litter_pose = node->Subscribe("/topic_litter_pose",litter_pose_cb);
	
	//Listen to repulsion values affecting robots
	gazebo::transport::SubscriberPtr topic_model_log_sub;
	topic_model_log_sub = node->Subscribe("/topic_model_log",topic_model_log_cb);
	
	gazebo::transport::SubscriberPtr sub_litter_in_nest;
	sub_litter_in_nest = node->Subscribe("/litter_in_nest",litter_in_nest_cb);
	
	gazebo::transport::SubscriberPtr sub_my_init;
	sub_my_init = node->Subscribe("/start_sim",start_sim_cb);
	
	gazebo::transport::SubscriberPtr sub_world_loaded;//subscriber to know when the world is ready to go
	sub_world_loaded = node->Subscribe("/world_loaded",sub_world_loaded_cb);
	
	
	gazebo::transport::SubscriberPtr sub_experiment_control;//readme file subscriber for start and end of simulation and simulation parameters used
	sub_experiment_control = node->Subscribe("/experiment_control",experiment_control_cb);
	
	//Publisher to indicate start of new simulation
	gazebo::transport::PublisherPtr pub_world_gov_experiment_control = node->Advertise<gazebo::msgs::Any>("/world_gov_experiment_control");

	//Publisher to reset and pause world.
	// gazebo::transport::PublisherPtr pub_world_control = node->Advertise<gazebo::msgs::Any>("/gazebo/default/world_control");
	// gazebo::msgs::WorldControl control_msg;
	// gazebo::msgs::WorldReset reset_msg;

	gazebo::transport::PublisherPtr pub_debug_governor = node->Advertise<gazebo::msgs::Any>("/debug_gov");
	gazebo::msgs::Any anyGov;
	anyGov.set_type(gazebo::msgs::Any::STRING);
	
	//for advertising simulation parameters
	//gazebo::transport::NodePtr node2(new gazebo::transport::Node());
	//node2->Init();
	gazebo::transport::PublisherPtr pub_params = node->Advertise<gazebo::msgs::Any>("/params_topic");
	//pub_params->WaitForConnection();
	//pub_params = node->Advertise<msgs::Any>("/communication_signal");
	//gazebo::msgs::Any any;
	//any.set_type(gazebo::msgs::Any::STRING);
	//std::string pars = "nei_sensing:5.0";
	//any.set_string_value(pars);
	gazebo::msgs::Any any;
	any.set_type(gazebo::msgs::Any::BOOLEAN);

	//Set step size of world
	// gazebo::msgs::Physics physicsMsg;
	// physicsMsg.set_type(gazebo::msgs::Physics::ODE);
	// double max_step_size_value = 0.025;
	// gazebo::transport::PublisherPtr physicsPub = node->Advertise<gazebo::msgs::Physics>("~/physics");
	
	while(true){//busy wait
		gazebo::common::Time::MSleep(100);
		time_t now = time(nullptr);
		double time_duration = difftime(now,message_time);
		anyGov.set_string_value(to_string(time_duration));
		pub_debug_governor->Publish(anyGov);

		if(world_loaded){
			if (time_duration > 10.0 and !world_gov_control_bool){
				//if world is idle for 10 seconds and world gov control value is false

				world_gov_control_bool = true;
				tt = std::time(nullptr);
				ttm = *std::localtime(&tt);
				
				any.set_bool_value(world_gov_control_bool);
				pub_world_gov_experiment_control->Publish(any);
				gazebo::msgs::Any exp_params;
				exp_params.set_type(gazebo::msgs::Any::STRING);
				if(param_it != my_params.end()){
					//If params iterator is not at the end, publish simulation parameters
					std::string sim_params = *param_it;
					
					algorithm_index = paramLine + param_it_loc;
					param_it++;//increment iterator to next parameter
					param_it_loc++;
					// std::cout<<sim_params<<std::endl;

					exp_params.set_string_value(sim_params);
					pub_params->Publish(exp_params);
				}
				else if(param_it == my_params.end()){
					//We are at the end of the simulation parameters vector. Exit simulation
					// std::cout<<"end simulation"<<std::endl;
					exp_params.set_string_value("end_simulation");
					pub_params->Publish(exp_params);
					break;
				}
				// control_msg.set_pause(false);

				// reset_msg.set_all(true);

				// control_msg.set_allocated_reset(&reset_msg);
				
				// pub_world_control->Publish(control_msg);

			}else if(!world_gov_control_bool){
				//if world control bool is set to false within experiment control, stop simulation
				any.set_bool_value(world_gov_control_bool);
				pub_world_gov_experiment_control->Publish(any);

				// any.set_bool_value(world_gov_control_bool);
				// pub_world_gov_experiment_control->Publish(any);

				// control_msg.set_pause(true);
				// reset_msg.set_all(true);

				// control_msg.set_allocated_reset(&reset_msg);
				
				// pub_world_control->Publish(control_msg);
			}
			
		}
		else{
			// physicsPub->Publish(physicsMsg);
		}
	}
	
	//Make sure to shut everything down
	gazebo::client::shutdown();
}