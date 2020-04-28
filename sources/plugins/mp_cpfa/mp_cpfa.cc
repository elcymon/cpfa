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
#include <ctime>

using namespace std;

namespace gazebo
{
    struct Utils{
        //Utility Functions
        double normalize(double angle){
            math::Angle dAngle(angle);
            dAngle.Normalize();
            return dAngle.Radian();
        }
        double dxy(math::Vector3 A, math::Vector3 B) {
            A.z = 0;
            B.z = 0;
            return A.Distance(B);
        }
    };
    struct ColorObject {
        msgs::Color *colMsg;
        msgs::Color *diffMsg;
        msgs::Visual visMsg;
        msgs::Material *materialMsg;
        gazebo::transport::PublisherPtr publishColor, visMsgPub;
        gazebo::transport::NodePtr node;
        gazebo::common::Color color;

        float red, green, blue, alpha;


        //constructor
        ColorObject(){
            //default constructor
        }
        ColorObject(double red, double green, double blue, double alpha){
            
            // gazebo::common::Color color(red,green,blue,alpha);
            // this->colMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(color));
            // this->diffMsg = new gazebo::msgs::Color(*(this->colMsg));
            this->red = red;
            this->green = green;
            this->blue = blue;
            this->alpha = alpha;

            this->node = transport::NodePtr(new transport::Node());
            this->node->Init();

            this->publishColor = this->node->Advertise<msgs::Visual>("/gazebo/default/visual");
            this->visMsgPub = this->node->Advertise<msgs::Visual>("/myVis");
        }
        void apply_color(std::string linkName, physics::ModelPtr modelPtr) {
            // std::cout<<"start apply on "<<modelPtr->GetName();

            // this->color = gazebo::common::Color(this->red,this->green,this->blue,this->alpha);
            // this->colMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(color));
            // this->diffMsg = new gazebo::msgs::Color(*(this->colMsg));


            gazebo::physics::LinkPtr link = modelPtr->GetLink(linkName);
            
            sdf::ElementPtr sdf = link->GetSDF();
            // // std::cout<<"\nOld Link: "<<link->GetSDF()->ToString("")<<std::endl;
            sdf = sdf->GetElement("visual");
            sdf = sdf->GetElement("material");
            sdf::ElementPtr ambientSDF = sdf->GetElement("ambient");
            sdf::ParamPtr paramPtr = ambientSDF->GetValue();
            gazebo::common::Color currentColor;
            paramPtr->Get(currentColor);
            // std::cout<<ccc.r<<ccc.g<<ccc.b<<std::endl;

            // sdf::ElementPtr sdf = link->GetSDF();
            // GZ_ASSERT(sdf->HasElement("visual"), "Malformed Link Element");
            // sdf = sdf->GetElement("visual");
            // GZ_ASSERT(sdf->HasAttribute("name"), "Malformed Visual element");
            // std::string visualName = sdf->Get<std::string>("name");

            this->visMsg = link->GetVisualMessage("visual");
            

            this->visMsg.set_name(link->GetScopedName());
            this->visMsg.set_parent_name(modelPtr->GetScopedName());

            this->materialMsg = this->visMsg.mutable_material();
            msgs::Color *ambient = this->materialMsg->mutable_ambient();
            if(currentColor.r == this->red && 
                currentColor.g == this->green && 
                currentColor.b == this->blue) {
                    //std::cout<<"No Color Change"<<std::endl;
                    return;
                }
            // this->visMsgPub->Publish(this->visMsg);
            ambient->set_r(this->red);
            ambient->set_g(this->green);
            ambient->set_b(this->blue);
            ambient->set_a(this->alpha);
            // std::cout<<visualName<<(ambient->r())<<(ambient->g())<<(ambient->b())<<" "<<
            //     this->red<<this->green<<this->blue<<std::endl;

            
            // sdf::ElementPtr diffuse = sdf->GetElement("diffuse");
            // sdf::ElementPtr specular = sdf->GetElement("specular");
            // sdf::ElementPtr emissive = sdf->GetElement("emissive");

            // ambient->Set(this->color);
            // diffuse->Set(this->color);
            // specular->Set(this->color);
            // emissive->Set(this->color);
            
            // modelPtr->UpdateParameters(modelPtr->GetSDF());
            // std::cout<<modelPtr->GetSDF()->ToString("")<<std::endl;
            // std::cout<<this->red<<this->green<<this->blue<< ambient->ToString("")<<std::endl;
            // std::cout<<"\nNew Link: "<<link->GetSDF()->ToString("")<<std::endl;
            // std::cout<<sdf->HasElement("ambient")<<std::endl;
            // this->materialMsg->clear_ambient();
            // this->materialMsg->clear_diffuse();
            // this->materialMsg->set_allocated_ambient(this->colMsg);
            // this->materialMsg->set_allocated_diffuse(this->diffMsg);
            msgs::Color *diffuse = this->materialMsg->mutable_diffuse();
            diffuse->set_r(this->red);
            diffuse->set_g(this->green);
            diffuse->set_b(this->blue);
            diffuse->set_a(this->alpha);

            // // std::cout<<" publish";
            this->publishColor->Publish(this->visMsg);
            // std::cout<<" finish color application"<<std::endl;
        }
    };
    

    struct RobotState {
        Utils utils;
        ColorObject detected;
        ColorObject undetected;

        double baseProb;
        double turnAmount;
        double desiredHeading;
        bool crashed;
        double xCrash;
        double yCrash;
        std::string action;//stop,forward,turn-right,turn-left,?reverse
        std::string state;//search,acquire,obstacle-avoidance,go-home
        
        gazebo::physics::ModelPtr nearestLitter;
        int seenLitter;
        int allLitterPicked;
        int maxCapacity;
        int currLitterPicked;
        double litterSensingRange;
        double fieldOfView;

        RobotState() {
            //default constructor
        }
        RobotState(double baseProb, double yaw, int maxCapacity,
                    double litterSensingRange, double fieldOfView) {
            this->baseProb = baseProb;
            this->desiredHeading = yaw;
            this->maxCapacity = maxCapacity;
            this->litterSensingRange = litterSensingRange;
            this->fieldOfView = fieldOfView;

            this->turnAmount = 0;
            this->crashed = false;
            this->xCrash = 0;
            this->yCrash = 0;
            this->action = "stop";
            this->state = "search";
            
            this->nearestLitter = nullptr;
            this->seenLitter = 0;
            this->allLitterPicked = 0;
            this->currLitterPicked = 0;

        }
        std::string state2string() {
            std::stringstream robotData;
            robotData <<"baseProb: "<<this->baseProb<<", "
                    <<"turnAmount: "<<this->turnAmount<<", "
                    <<"desiredHeading: "<<this->desiredHeading<<", "
                    <<"crashed: "<<this->crashed<<", "
                    <<"xCrash: "<<this->xCrash<<", "
                    <<"yCrash: "<<this->yCrash<<", "
                    <<"action: "<<this->action<<", "
                    <<"state: "<<this->state<<", "
                    <<"seenLitter: "<<this->seenLitter<<", "
                    <<"allLitterPicked: "<<this->allLitterPicked<<", "
                    <<"maxCapacity: "<<this->maxCapacity<<", "
                    <<"currLitterPicked: "<<this->currLitterPicked<<", "
                    <<"litterSensingRange: "<<this->litterSensingRange<<", "
                    <<"fieldOfView: "<<this->fieldOfView//<<", "
                    ;
            return robotData.str();
        }

    };
    struct RW {
        Utils utils;
        std::default_random_engine generator;
        std::uniform_real_distribution<double> uformRand;
        std::normal_distribution<double> normalRandHeading;
        RW(std::default_random_engine generator,
            std::uniform_real_distribution<double> uformRand,
            std::normal_distribution<double> normalRandHeading) {

                this->generator = generator;
                this->uformRand = uformRand;
                this->normalRandHeading = normalRandHeading;
        }
        RW() {
            //default constructor
        }

        void updateHeading (RobotState* currState){
            
            if(this->uformRand(this->generator) < currState->baseProb)
            {//randomly change desired direction
                double tempTurnAmount = this->normalRandHeading(this->generator);
                if(tempTurnAmount > 2 * M_PI) tempTurnAmount = 2 * M_PI;

                if(tempTurnAmount < 0.0) tempTurnAmount = 0.0;

                tempTurnAmount = (this->utils).normalize(tempTurnAmount);
                currState->desiredHeading = currState->desiredHeading + tempTurnAmount;
            }
            
        }
                
    };

    // struct CPFA {
    //     /* This class implements the CPFA algorithm */
    // };
	class MP_CPFA : public ModelPlugin
	{
		private:
            std::mutex mutex;
            physics::ModelPtr model;
            physics::JointPtr rwheel;
            physics::JointPtr lwheel;

            double rVel;
            double Kp;
            double wheel_separation;
            RobotState myState;
            RW randomWalk;
            Utils utils;
            
            transport::NodePtr node;
            
            transport::SubscriberPtr contactSub;
            transport::PublisherPtr pubRobotData;
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
            this->wheel_separation = this->rwheel->GetAnchor(0).Distance(
                                        this->lwheel->GetAnchor(0));
            
            this->node = transport::NodePtr(new transport::Node());
            this->node->Init();
            
            std:: string contactMsg = "/gazebo/default/" + this->model->GetName() + 
                "/chassis/chassis_contact/contacts";
            this->contactSub = this->node->Subscribe(contactMsg, &MP_CPFA::ContactCB,this);
            this->pubRobotData = this->node->Advertise<msgs::Any>(this->model->GetName() + "/data");

            //Initialize Parameters
            this->initParams();

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&MP_CPFA::OnUpdate, this, _1)
            );
        }
        public: void ContactCB(ConstContactsPtr &c)
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            math::Quaternion myRot = (this->model->GetWorldPose()).rot;
            double headingError = (this->myState).desiredHeading - myRot.GetYaw();
            headingError = (this->utils).normalize(headingError);

            if( !((this->myState).crashed) && abs(headingError) < 0.09)
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
                                (this->myState).xCrash = contacts.contact(i).position(j).x();
                                (this->myState).yCrash = contacts.contact(i).position(j).y();
                                (this->myState).turnAmount = this->avoidObstacle((this->myState).xCrash,
                                    (this->myState).yCrash, this->model->GetWorldPose());
                                (this->myState).crashed = true;
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
            dtheta = (this->utils).normalize(dtheta);

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
                    turnRequired = (this->utils).normalize(x);
                }
                
            }
            return turnRequired;
        }
        
        public: void litterSensor(RobotState* myState, 
                                gazebo::physics::ModelPtr myModel, 
                                gazebo::physics::WorldPtr myWorld) {
            //sense presence of litter and update robotState
            gazebo::physics::Model_V models = myWorld->GetModels();

            gazebo::math::Pose my_pose = myModel->GetWorldPose();
            gazebo::math::Vector3 my_pos = my_pose.pos;
            double my_yaw = my_pose.rot.GetYaw();
            int seenLter = 0;
            // std::cout<<myModel->GetName()<<" ";
            for (auto m : models) {//search through models in world
                std::string m_name = m->GetName();
                if(m_name.find("litter") != std::string::npos) {
                    gazebo::math::Vector3 m_pos = m->GetWorldPose().pos;
                    double dist = (this->utils).dxy(my_pos,m_pos);//linear distance
                    double lit_or = (this->utils).normalize(atan2(my_pos.y - m_pos.y, my_pos.x - m_pos.x))
                                    - my_yaw; //orientation of litter wrt robot
                    
                    if((dist <= myState->litterSensingRange) and //linear distance within sensing range
                        (lit_or >= -myState->fieldOfView/2 and lit_or <= myState->fieldOfView/2)){//and within field of view
                        //litter within sensing range
                        seenLter += 1;//increment litter

                        //START: debugging litter seen
                        std::string myName = myModel->GetName();
                        if (myName.find("robot18") != std::string::npos) {

                            (this->myState).detected.apply_color("link", m);
                        }
                        
                    }
                    else {//not within detection range
                        std::string myName = myModel->GetName();
                        if (myName.find("robot18") != std::string::npos) {
                            (this->myState).undetected.apply_color("link", m);
                        }
                    }//END: debugging litter seen
                }
            }
            myState->seenLitter = seenLter;//update litter count
            // std::cout<<"sensed";
        }
        public : void initParams()
        {
            this->rVel = 10.0;
            // (this->myState).baseProb = 0.0025;
            // (this->myState).turnAmount = 0;
            
            math::Quaternion myRot = (this->model->GetWorldPose()).rot;
            // (this->myState).desiredHeading = myRot.GetYaw();
            
            // (this->myState).crashed = false;
            
            this->Kp = 10 * this->rVel;
            
            // (this->myState).xCrash;
            // (this->myState).yCrash;
            
            // the seed for the random number generation
            std::string rName = this->model->GetName();
            int robNum = stoi(rName.substr(9));
            std::time_t currTime = std::time(nullptr);
            robNum = (int)  currTime / robNum;
            // int xPos = (int) abs((this->model->GetWorldPose()).pos.x);
            // int yPos = (int) abs((this->model->GetWorldPose()).pos.y);
            this->generator = std::default_random_engine(robNum);

            this->uformMin = 0;
            this->uformMax = 1;
            this->uformRand = std::uniform_real_distribution<double>(this->uformMin,this->uformMax);
            

            this->normalStddev = 1.5707;
            this->normalMean = 3.142;
            this->normalRandHeading = std::normal_distribution<double>(this->normalMean,this->normalStddev);

            //initialize state object            
            this->myState = RobotState(0.0025,myRot.GetYaw(),1000,15,M_PI * 2);
            //update algorithm object
            this->randomWalk = RW(this->generator,this->uformRand,this->normalRandHeading);

            (this->myState).undetected = ColorObject(1,0,0,1);
            (this->myState).detected = ColorObject(0,0,1,1);
        }
        public: std::string rotate(double headingError)
        {
            std::string dxn;
            if(headingError > 0)
            {
                this->lwheel->SetVelocity(0,-(this->rVel/2.0));
                this->rwheel->SetVelocity(0,this->rVel/2.0);
                dxn = "turn-left";
            }
            else if(headingError < 0)
            {
                this->lwheel->SetVelocity(0,(this->rVel/2.0));
                this->rwheel->SetVelocity(0,-(this->rVel/2.0));
                dxn = "turn-right";
            }
            return dxn;
        }

        public: void moveForward(double headingError)
        {
            double va = headingError * this->Kp;
            double r = this->rVel + va * this->wheel_separation / 2.0;
            double l = this->rVel - va * this->wheel_separation / 2.0;

            if (r > this->rVel)
                r = this->rVel;
            if (r < 0)
                r = 0;

            if (l > this->rVel)
                l = this->rVel;
            if (l < 0)
                l = 0;
            
            this->rwheel->SetVelocity(0,r);
            this->lwheel->SetVelocity(0,l);
        }

        public: void OnUpdate(const common::UpdateInfo & _info)
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            // std::cout<<'start ';
            math::Quaternion myRot = (this->model->GetWorldPose()).rot;
            // (this->myState).desiredHeading = myRot.GetYaw();
            // std::stringstream robotData;

            //sense presence of litter
            this->litterSensor(&(this->myState), this->model, this->model->GetWorld());

            
            if((this->myState).crashed)
            {//avoid obstacle: change direction; state=obstacle-avoidance
                (this->myState).desiredHeading = (this->myState).desiredHeading + (this->myState).turnAmount;
                (this->myState).crashed = false;
                (this->myState).state = "obstacle-avoidance";

                //robotData<<"crashed: "<<(this->myState).desiredHeading<<",";
            }
            // else if() {
            //     //capacity is full, to toward home: state=go-home
            // }
            // else if() {
            //     //litter found, go toward closest litter: state=acquire
            // }
            else {
                //Execute Control Algorithm: state=search; RW, 
                (this->randomWalk).updateHeading(&(this->myState));
                
                (this->myState).state = "search";
                //robotData<<"Random turn: "<<(this->myState).desiredHeading<<",";                
            }

            (this->myState).desiredHeading = (this->utils).normalize((this->myState).desiredHeading);
            double headingError = (this->myState).desiredHeading - myRot.GetYaw();
            headingError = (this->utils).normalize(headingError);
            if(abs(headingError) < 0.09)
            {//move straight
                this->moveForward(headingError);
                (this->myState).action = "forward";
                //robotData<<"move straight,";
            }
            else
            {
                //turn in desired direction
                (this->myState).action = this->rotate(headingError);
                
            }
            // robotData<<(this->myState).state<<","<<(this->myState).action<<": "
            //     <<(this->myState).desiredHeading<<" - "<<myRot.GetYaw()<<" = "
            //     <<headingError<<std::endl;
            std::string robotDataStr = (this->myState).state2string();
            msgs::Any robotDataMsg;
            robotDataMsg.set_type(msgs::Any::STRING);
            robotDataMsg.set_string_value(robotDataStr);
            // robotDataMsg.set_string_value(robotData.str());
            this->pubRobotData->Publish(robotDataMsg);
            std::string myName  = this->model->GetName();
            if (myName.find("robot18") != std::string::npos) {
              std::cout<<"seen litter: "<< (this->myState).seenLitter<<std::endl;
            }
            // std::cout<<' finish'<<std::endl;
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MP_CPFA)
}

