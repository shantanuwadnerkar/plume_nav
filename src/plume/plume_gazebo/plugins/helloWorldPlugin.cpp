#include <iostream>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include"ros/ros.h"

namespace gazebo
{
class helloWorldPlugin : public WorldPlugin
{
public:
    helloWorldPlugin() : WorldPlugin()
            {
                ROS_INFO("Hello World!");
            }


    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*sdf*/)
    {
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        std::cout << "Hellooooo!" << std::endl;
    }
};

GZ_REGISTER_WORLD_PLUGIN(helloWorldPlugin)

}
