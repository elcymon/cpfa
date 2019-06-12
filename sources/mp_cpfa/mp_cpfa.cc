#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <iostream> //needed in order to use cout
#include <mutex>
#include <unistd.h>
using namespace std;

namespace gazebo
{
	class MP_CPFA : public ModelPlugin
	{
		private:
            std::mutex mutex;
            physics::JointPtr rwheel;
            physics::JointPtr lwheel;

            double rVel;

            transport::NodePtr node;
            
            double desiredHeading;
            std::string contactMsg;
            transport::SubscriberPtr contactSub;
            bool crashed;
            double xCrash;
            double yCrash;
            // when robot has to go in a particular direction, it needs to make a detour to avoid obstacle


            //WORKING WITH RANDOM NUMBERS
            std::default_random_engine generator;
            std::uniform_real_distribution<double> uformRand;
            std::uniform_real_distribution<double>uformRandHeading;
            double uformMin, uformMax;
            std::normal_distribution<double> normalRandHeading;
            double normalStddev, normalMean;

            event::ConnectionPtr updateConnection;


        public : void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            //     boost::bind(&Nest_Plugin::OnUpdate, this, _1)
            // );
        }
        public: void OnUpdate(const common::UpdateInfo & _info)
        {
            //std::lock_guard<std::mutex> lock(this->mutex);
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MP_CPFA)
}
