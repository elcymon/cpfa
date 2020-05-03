#ifndef ROBOT_HH
#define ROBOT_HH
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <string>
#include <iostream>
#include <map>
namespace gazebo {
    class Robot {
        private:
            physics::ModelPtr model;
            physics::JointPtr rwheel;
            physics::JointPtr lwheel;

            double velocity;
            double Kp;
            double wheel_separation;
            std::string state;

            transport::SubscriberPtr sub_contact_sensor;
        
        public:
            Robot(physics::ModelPtr m, const std::map<std::string,std::string> &params);
            
            void Straight( double heading_error );
            std::string Turn( double heading_error);
            void Stop ( );
            double AvoidObstacle(double x, double y);
            void CB_ContactSensor(ConstContactsPtr &c);


    };
}
#endif