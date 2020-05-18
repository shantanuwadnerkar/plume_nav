#include <iostream>
#include <thread>
// #include <chrono>

#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
// #include <gazebo/common/Plugin.hh>

#include"ros/ros.h"

namespace gazebo
{
class plumeSimulatorPlugin : public WorldPlugin
{
public:
    plumeSimulatorPlugin() : WorldPlugin()
            {
                ROS_INFO("Spawn sphere plugin initialized!");
            }
    
    double totalSimulationTime{10};
    double filamentPerSimulator{1}; 

    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*sdf*/)
    {        
        sdf::SDF sphereSDF;
        sphereSDF.SetFromString(
        "<sdf version ='1.6'>\
            <model name ='sphere'>\
            <static>true</static>\
                <pose>0 0 1 0 0 0</pose>\
                <link name ='link'>\
                    <self_collide>false</self_collide>\
                    <pose>0 0 0 0 0 0</pose>\
                    <collision name='collision'>\
                    <geometry>\
                    <sphere><radius>0.01</radius></sphere>\
                    </geometry>\
                    <surface>\
                        <contact><collide_bitmask>0x01</collide_bitmask></contact>\
                    </surface>\
                    </collision>\
                    <visual name ='visual'>\
                    <geometry>\
                    <sphere><radius>0.01</radius></sphere>\
                    </geometry>\
                    </visual>\
                </link>\
            </model>\
        </sdf>");
        
        int i{0};

        // std::vector<sdf::ElementPtr> modelVector;
        // resize vec here with totalSimulationTime * filamentPerSimulator

        for (0; i<totalSimulationTime*filamentPerSimulator; i++)
        {
            std::stringstream ssVarName;
            std::stringstream ssHex;
            ssVarName << "filament_" << i;
            std::string varName = ssVarName.str();

            sdf::ElementPtr modelsdfPtr = sphereSDF.Root();
            modelsdfPtr->GetElement("model")->GetAttribute("name")->SetFromString(varName);
            
            ssHex << "0x" << std::hex << i+1;
            std::cout << ssHex.str();
            sdf::ElementPtr collisionPtr = modelsdfPtr->GetElement("model")->GetElement("link")->GetElement("collision");
            sdf::ElementPtr contactPtr = collisionPtr->GetElement("surface")->GetElement("contact");
            sdf::ParamPtr collideBitmaskPtr = contactPtr->GetElement("collide_bitmask")->GetValue();
            bool setCollideBitmask = contactPtr->GetElement("collide_bitmask")->Set(ssHex.str());

            // modelVector.push_back(modelsdfPtr);
            _parent->InsertModelSDF(sphereSDF);

        }

        ROS_INFO("Sphere initializing...");

        // for (std::vector<sdf::ElementPtr>::const_iterator i = modelVector.begin(); i != modelVector.end(); ++i)
        //     std::cout << *i << ' ';

        // ROS_INFO("Sphere initialized!");
        // std::chrono::milliseconds timespan(2000); // or whatever
        // std::this_thread::sleep_for(timespan);
        // ROS_INFO("Sleep over!");
    }
};

GZ_REGISTER_WORLD_PLUGIN(plumeSimulatorPlugin)

}
