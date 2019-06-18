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
            physics::ModelPtr model;
            physics::JointPtr rwheel;
            physics::JointPtr lwheel;

            double rVel;
            double baseProb;
            double turnAmount;

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
            double uformMin, uformMax;
            std::normal_distribution<double> normalRandHeading;
            double normalStddev, normalMean;

            event::ConnectionPtr updateConnection;


        public : void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;
            
            this->rwheel = this->model->GetJoint("R_joint");
            this->lwheel = this->model->GetJoint("L_joint");
            
            this->node = transport::NodePtr(new transport::Node());
            this->node->Init();
            
            this->contactMsg = "/gazebo/default/" + this->model->GetName() + 
                "/chassis/chassis_contact/contacts";
            this->contactSub = this->node->Subscribe(this->contactMsg, &MP_CPFA::ContactCB,this);

            //Initialize Parameters
            this->initParams();

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&MP_CPFA::OnUpdate, this, _1)
            );
        }
        public: void ContactCB(ConstContactsPtr &c)
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            if( !(this->crashed))
            {
                msgs::Contacts contacts = *c;
                for(int i = 0; i < (int) contacts.contact_size(); i++)
                {
                    std::string col1 = contacts.contact(i).collision1();
                    std::string col2 = contacts.contact(i).collision2();
                    std::string collision = col1 + " " + col2;

                    bool _me = (collision.find(this->model->GetName()) != std::string::npos);
                    bool _obstacle = collision.find("chassis::collision") != std::string::npos;
                    bool _litter = collision.find("litter") != std::string::npos;

                    if(_me && _obstacle && not _litter)
                    {
                        for (int j = 0; j < (int) contacts.contact(i).position_size(); j++)
                        {
                            if (contacts.contact(i).position(j).z() > 0.004)
                            {
                                this->xCrash = contacts.contact(i).position(j).x();
                                this->yCrash = contacts.contact(i).position(j).y();
                                this->turnAmount = this->avoidObstacle(this->xCrash,
                                    this->yCrash, this->model->GetWorldPose());
                                this->crashed = true;
                                return;
                            }
                        }
                    }
                }
                
            }
        }
        public: double avoidObstacle(double x, double y, math::Pose myPos)
        {
            double turnRequired = 0;
            double yaw = myPos.rot.GetYaw();
            double xc = myPos.pos.x;
            double yc = myPos.pos.y;
            double obstacleLoc = atan2(y - yc, x - xc);
            double dtheta = yaw - obstacleLoc;
            dtheta = this->normalize(dtheta);

            if(abs(dtheta) < M_PI / 2.0)
            {
                if(dtheta < - M_PI / 6.0)
                {
                    turnRequired = - M_PI / 4.0;
                }
                else if(dtheta > M_PI / 6.0)
                {
                    turnRequired = M_PI / 4.0;
                }
                else
                {
                    double x = this->uformRand(this->generator) * M_PI + M_PI / 2.0;
                    turnRequired = this->normalize(x);
                }
                
            }
            return turnRequired;
        }
        
        public : double normalize(double angle)
        {
            math::Angle dAngle(angle);
            dAngle.Normalize();
            return dAngle.Radian();
        }

        public : void initParams()
        {
            this->rVel = 10.0;
            this->baseProb = 0.0025;
            this->turnAmount = 0;
            
            this->desiredHeading = 0;
            
            this->crashed = false;
            
            // this->xCrash;
            // this->yCrash;
            
            // the seed for the random number generation
            double xPos = abs(this->model->GetWorldPose().pos.x);
            double yPos = abs(this->model->GetWorldPose().pos.y);
            this->generator = std::default_random_engine( xPos * yPos);

            this->uformMin = 0;
            this->uformMax = 1;
            this->uformRand = std::uniform_real_distribution<double>(this->uformMin,this->uformMax);
            

            this->normalRandHeading = std::normal_distribution<double>(this->normalMean,this->normalStddev);
            
            this->normalStddev = 1.5707;
            this->normalMean = 3.142;
        }
        public: void OnUpdate(const common::UpdateInfo & _info)
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            if(this->crashed)
            {//avoid obstacle: change direction

            }
            else {
                //continue random walk
                if(this->uformRand(this->generator) < this->baseProb)
                {//randomly change desired direction

                }
            }
            if(this->turnAmount < 0.09)
            {//move straight

            }
            else
            {
                //turn in desired direction
            }
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MP_CPFA)
}
