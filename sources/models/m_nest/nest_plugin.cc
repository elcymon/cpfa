#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
//#include <gazebo/sdf/sdf.hh>
#include <iostream> //needed in order to use cout
#include <mutex>
#include <unistd.h>
using namespace std;

namespace gazebo
{
	class Nest_Plugin : public ModelPlugin
	{
		private:
			std::vector<std::string> litter_count;
			transport::NodePtr node;
			transport::SubscriberPtr sub;
			transport::PublisherPtr pub;
			transport::SubscriberPtr sub_my_Init;
			transport::SubscriberPtr sub_start_sim;
			std::mutex mutex;
			
			physics::ModelPtr nest_model;
			physics::JointPtr rwheel;
			physics::JointPtr lwheel;
			double nVel;
			double wheel_separation;
			int mvStop;//delete
			double desired_heading;
			double nest_diameter;
			double log_timer;
			double max_step_size;
			double log_rate;
			bool start_sim;
			bool initialized;//nest initialized

			gazebo::physics::Model_V robots;
			gazebo::physics::Model_V litters;

			//needed for to and fro exploration of nest
			std::vector<int> xPoints;
			std::vector<int> yPoints;
			std::vector<int>::iterator xIt;
			std::vector<int>::iterator yIt;

			gazebo::math::Vector3 goal_checkpoint;
			//switch x and y then reverse points order
			bool switchedXY;

			//keep track of distance travelled by nest
			gazebo::math::Pose startPose;//robot start location
			gazebo::math::Pose prevPose;
			double distance;

			// Pointer to the update event connection
			 event::ConnectionPtr updateConnection;
		
		public : void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			
			//initialize exploration checkpoints
			this->xPoints = {10,90,90, 10, 10, 90, 90, 10, 10, 90};
			this->xIt = this->xPoints.begin();

			this->yPoints = {10, 10,30, 30,50, 50, 70, 70, 90, 90};
			this->yIt = this->yPoints.begin();

			//initially  iterators are not switched
			this->switchedXY = false;

			
			//Initialize the litter vector
			
			//create a subscriber to subscribe to the litter topic
			//Remember to make all robots send litter names to that topic
			
			this->node = transport::NodePtr(new transport::Node());
			this->node->Init();
			this->sub = this->node->Subscribe("/litter_deposits",&Nest_Plugin::Collect_Litter,this);
			this->pub = this->node->Advertise<msgs::Any>("/litter_in_nest");
			this->start_sim = false;

			this->nest_model = _parent;
			this->rwheel = this->nest_model->GetJoint("R_joint");
			this->lwheel = this->nest_model->GetJoint("L_joint");
			this->wheel_separation = this->rwheel->GetAnchor(0)
								.Distance(this->lwheel->GetAnchor(0));
			math::Box chassis = this->nest_model->GetLink("chassis")->GetBoundingBox();
			this->nest_diameter = chassis.GetXLength();
			

			this->sub_my_Init = this->node->Subscribe("/my_Init",&Nest_Plugin::my_Init,this);
			this->sub_start_sim = this->node->Subscribe("/start_sim",&Nest_Plugin::start_sim_cb,this);

			this->initialized = false;
			//get pointer to robots in the swarm
			gazebo::physics::Model_V all_models = this->nest_model->GetWorld()->GetModels();
			for (auto m : all_models) {
				std::string model_name = m->GetName();
				if(model_name.find("m_4wrobot") != std::string::npos) {
					this->robots.push_back(m);
				}
				else if (model_name.find("litter") != std::string::npos) {
					this->litters.push_back(m);
				}
			}
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&Nest_Plugin::OnUpdate, this, _1));
		}

		public: void stop() {
			this->lwheel->SetVelocity(0,0);
			this->rwheel->SetVelocity(0,0);
		}
		public: void headingControl(double dxn_eror)
		{/*
			Takes in error in heading and adjust wheel velocities to minimize the error
		*/
			double l = 0, r = 0;
				if(abs(dxn_eror) < 0.09){
					double va = dxn_eror * 100 * this->nVel;
					r = this->nVel + va * this->wheel_separation/2;
					l = this->nVel - va * this->wheel_separation/2;

					if (r > this->nVel)
						r = this->nVel;
					if (r < 0)
						r = 0;

					if (l > this->nVel)
						l = this->nVel;
					if (l < 0)
						l = 0;
				}
				else {
						if(dxn_eror > 0)
						{//Object on the right. Turn right
							l = -(this->nVel/2);
							r = this->nVel/2;
							
						}
						else if(dxn_eror < 0)
						{//Object on left. Turn left.
							l = this->nVel/2;
							r = -(this->nVel/2);
						}
				}
			
			
			//set robot velocities
			this->rwheel->SetVelocity(0,r);
			this->lwheel->SetVelocity(0,l);
		}
		public: double normalize(double angle)
		{//normalize angle
			math::Angle D_angle(angle);
			D_angle.Normalize();
			return D_angle.Radian();
		}
		public: bool checkNestFrontestFront() {
		
			bool frontClear = true;
			gazebo::math::Pose myPose = this->nest_model->GetWorldPose();
			double nest_heading = myPose.rot.GetYaw();

			math::Vector3 nest_pos = myPose.pos;
			nest_pos.z = 0;//We are interested only in x,y distance from the nest

			gazebo::physics::Model_V all_models = this->nest_model->GetWorld()->GetModels();
			for (auto m : all_models) {
				std::string model_name = m->GetName();
				if(model_name.find("m_4wrobot") != std::string::npos) {
					//compute distance
					math::Vector3 rob_pos = m->GetWorldPose().pos;
					rob_pos.z = 0;
					
					double distance = nest_pos.Distance(rob_pos);

					//compute orientation of robot to nest
					double obstruction_orientation = atan2(rob_pos.y - nest_pos.y,
														rob_pos.x - nest_pos.x);
					double relative_orientation = nest_heading - obstruction_orientation;
					relative_orientation = this->normalize(relative_orientation);

					if(abs(relative_orientation) < M_PI/2.0 and
									distance < (this->nest_diameter))
					{//if obstruction is in front and less than nest diameter.
						frontClear = false;

					}


				}
			}

			return frontClear;
		}

		public: void my_Init(ConstAnyPtr &any)
		{//used to initialize/reset simulation parameters
			std::lock_guard<std::mutex> lock(this->mutex);
			std::string sim_params_mine = any->string_value();
			
			while(sim_params_mine.find(",") != std::string::npos)
			{
				std::size_t ploc = sim_params_mine.find(",");
				std::string temp = sim_params_mine.substr(0,ploc);
				
				std::size_t aloc = temp.find(":");
				std::string param_name = temp.substr(0,aloc);
				std::string param_value_str = temp.substr(aloc+1);
				// double value = std::stod(param_value_str);
				
				if(param_name.compare("log_rate") == 0)
				{
					this->log_rate = std::stod(param_value_str);;
				}
				else if(param_name.compare("nVel") == 0) {
					this->nVel = std::stod(param_value_str);
				}
				else if(param_name.compare("max_step_size") == 0)
				{
					this->max_step_size = std::stod(param_value_str);;
				}
				else
				{
				}
				sim_params_mine = sim_params_mine.substr(ploc+1);
			}

			this->mvStop = -1;
			this->desired_heading = 0;//move in positive x direction

			this->log_timer = 0;//reset timer
			this->startPose = this->nest_model->GetWorldPose();
			this->startPose.pos.z = 0;//only interested in xy distance
			this->prevPose = this->startPose;
			this->distance = 0;//initialize distance travelled as zero

			//log data header
			std::stringstream swarm_data;
			
			swarm_data << "t,litter_count,nest_x,nest_y,nest_yaw,nest_dst";
			//robots header
			for (auto robot : this->robots) {
				std::string rName = robot->GetName();
				swarm_data << "," << rName <<"_x," << rName << "_y," << rName << "_dst";
			}

			//targets/litter header
			for (auto l : this->litters) {
				std::string lName = l->GetName();
				swarm_data << "," << lName << "_x," << lName << "_y";
			}
			

			//intitialize first checkpoint
			this->goal_checkpoint.z = 0;
			this->goal_checkpoint.y = *(this->yIt);
			this->goal_checkpoint.x = *(this->xIt);

			msgs::Any b;
			b.set_type(msgs::Any::STRING);
			b.set_string_value(swarm_data.str());
			this->pub->Publish(b);
			this->initialized = true;
		}
		
		public: void start_sim_cb(ConstAnyPtr &any)
		{//receives true when simulation is to start and false when simulation stops
			std::lock_guard<std::mutex> lock(this->mutex);
			this->start_sim = any->bool_value();
			if(this->start_sim)
			{//start sim is true, that means a new simulation has started.
				//Therefore, reset litter count.
				this->litter_count.clear();
				
			}
		}
		
		public: void Collect_Litter(ConstAnyPtr &any)
		{
			std::lock_guard<std::mutex> lock(this->mutex);
			msgs::Any a = *any;
			std::string litter_name = a.string_value();
			this->litter_count.push_back(litter_name);
			//msgs::Any b;
			//b.set_type(msgs::Any::STRING);
			//b.set_string_value(to_string(this->litter_count.size()));
			//this->pub->Publish(b);
			//Use this to add litter objects to the vector.
			//Remember what happened when picking litter with contact sensor
			//If names of litter appear multiple times make use of set instead of vector
			//i.e. change the declaration from vector to set container.
		}
		
		// Called by the world update start event
		public: void OnUpdate(const common::UpdateInfo & _info)
		{
			std::lock_guard<std::mutex> lock(this->mutex);
			this->log_timer += this->max_step_size;
			if(this->start_sim and this->initialized){
				//compute the heading error of the nest
				gazebo::math::Pose myPose = this->nest_model->GetWorldPose();
				myPose.pos.z = 0;

				double checkpoint_dist = this->goal_checkpoint.Distance(myPose.pos);
				if (checkpoint_dist < 0.05) {

					
					if(this->switchedXY)
					{//if switched, move in reverse order
						this->yIt--;
						this->xIt--;
						if(this->yIt == this->xPoints.begin())
						{//beginning of point reached. Switch again
							this->switchedXY = false;
							this->yIt = this->yPoints.begin();
							this->xIt = this->xPoints.begin();
						}
					}
					else{//correct motion through the points from beginning to end
						this->yIt++;
						this->xIt++;

						if(this->yIt == this->yPoints.end()){
							//switch points if end is reached
							this->switchedXY = true;
							this->yIt = this->xPoints.end();
							this->xIt = this->yPoints.end();
							//end is an invalid point, so decrement immediately
							this->yIt--;
							this->xIt--;
						}
					}
					//update checkpoint
					this->goal_checkpoint.y = *(this->yIt);
					this->goal_checkpoint.x = *(this->xIt);
					
				}

				//get next checkpoint pos
				double checkpoint_loc = atan2(this->goal_checkpoint.y - myPose.pos.y,
											this->goal_checkpoint.x - myPose.pos.x);
				
				this->desired_heading = this->normalize(checkpoint_loc);
				//compute checkpoint orientation

				double heading_error = this->desired_heading - 
										myPose.rot.GetYaw();
				heading_error = this->normalize(heading_error);
				
				//keep moving straight unless about to hit a robot.
				//check if there are robots obstructing nest's path
				bool frontClear = true;//checkNestFrontestFront();//compute frontclear inline

				std::stringstream swarm_data;
				for (auto robot : this->robots) {
					gazebo::math::Pose robPose = robot->GetWorldPose();
					robPose.pos.z = 0;
					
					//compute distance
					double robDist = myPose.pos.Distance(robPose.pos);

					//log robot data
					swarm_data << "," << robPose.pos.x << "," <<robPose.pos.y << "," <<robDist;

					//compute orientation of robot to nest
					double obstruct_orientation = atan2(robPose.pos.y - myPose.pos.y,
														robPose.pos.x - myPose.pos.x);

					double relative_orientation = myPose.rot.GetYaw() - obstruct_orientation;
					relative_orientation = this->normalize(relative_orientation);

					if(abs(relative_orientation) < M_PI / 2.0 and
							robDist < (this->nest_diameter)) {
						//if obstruction is in front and less than nest diameter.
						frontClear = false;
					}
				}

				//litter/targets location data
				for(auto l : this->litters) {
					gazebo::math::Vector3 lPos = l->GetWorldPose().pos;
					swarm_data << "," << lPos.x << "," << lPos.y;
				}
				if(frontClear) {
					this->headingControl(heading_error);
				}
				else { //if front is not clear, stop
					this->stop();
				}



				gazebo::common::Time st = _info.simTime;
				// if(_info.simTime.sec % 10 == 0 and _info.simTime.nsec==0){
				// 	this->mvStop *= -1;
				// }

				// if(this->mvStop > 0) {
				// 	this->stop();
				// }
				// else {
				// 	gazebo::math::Pose myPose = this->nest_model->GetWorldPose();
				// 	double heading_error = this->desired_heading - 
				// 							myPose.rot.GetYaw();
				// 	heading_error = this->normalize(heading_error);
				// 	this->headingControl(heading_error);
				// }
				
				//compute nest distance travelled
				myPose.pos.z=0;
				//increment total distance travelled
				this->distance = this->distance + myPose.pos.Distance(this->prevPose.pos);
				this->prevPose = myPose;//update prevpose to be used in next time step
				
				if(/*st.nsec==0 and this->start_sim)//or */(this->log_timer >= this->log_rate ))//rate of 100Hz
				{
					this->log_timer = 0;
					double dx = round(this->distance * 100.0) / 100.0;

					std::string log_litter_count(to_string(_info.simTime.Double()));
					log_litter_count += "," + to_string(this->litter_count.size()) +
										"," + to_string(myPose.pos.x) + "," + to_string(myPose.pos.y) +
										"," + to_string(myPose.rot.GetYaw()) + "," + to_string(dx)
										+ swarm_data.str();
					msgs::Any b;
					b.set_type(msgs::Any::STRING);
					b.set_string_value(log_litter_count);
					this->pub->Publish(b);
				}
			}
			
			if(this->log_timer > this->log_rate)
			{
				this->log_timer = 0;
			}
		}
		
		
	};
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(Nest_Plugin)
}
