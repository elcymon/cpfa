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
	class Litter_Plugin : public ModelPlugin
	{
		private:
			transport::NodePtr node;
			std::mutex mutex;
			transport::PublisherPtr pub;
			physics::ModelPtr model;
			bool send;

			 event::ConnectionPtr updateConnection;
		
		public : void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			this->model = _parent;
			this->send = true;

			this->node = transport::NodePtr(new transport::Node());
			this->node->Init();
			std::string topic = "/litter_data";
			this->pub = this->node->Advertise<msgs::Any>(topic);

			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&Litter_Plugin::OnUpdate, this, _1));
		}

		
		// Called by the world update start event
		public: void OnUpdate(const common::UpdateInfo & _info)
		{
			std::lock_guard<std::mutex> lock(this->mutex);
			if(this->send)
			{
				this->send = false;
				msgs::Pose p = msgs::Convert(this->model->GetWorldPose().Ign());
				p.set_name(this->model->GetName());
				msgs::Any any;
				any.set_type(msgs::Any::POSE3D);
				any.mutable_pose3d_value()->CopyFrom(p);
				this->pub->Publish(any);
			}
			
		}
		
	};
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(Litter_Plugin)
}
