#include <iostream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


class VelocityTransform
{
public:
    VelocityTransform();
    ~VelocityTransform();
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    
    double x{0.0};
    double y{0.0};
    double z{0.0};

    private:
};

VelocityTransform::VelocityTransform()
{}

VelocityTransform::~VelocityTransform()
{}

void VelocityTransform::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("OOP");
    x = msg->linear.x;
    y = msg->linear.y;
    z = msg->linear.z;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_transform");
    ros::NodeHandle n;

    VelocityTransform velocityTransform;

    tf::TransformBroadcaster mapToOdom_;
    tf::Transform map_transform_{ tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0) };

    tf::TransformBroadcaster odomTransform_;
    geometry_msgs::TransformStamped odom_transform;

    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1, &VelocityTransform::cmd_vel_callback, &velocityTransform);

    ros::Rate rate(100);

    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_link";

    while (ros::ok())
    {
        mapToOdom_.sendTransform(tf::StampedTransform(map_transform_, ros::Time::now(), "map", "odom"));        

        // ROS_INFO("%.2f, %.2f, %.2f", velocityTransform.x, velocityTransform.y, velocityTransform.z);

        odom_transform.header.stamp = ros::Time::now();
        odom_transform.transform.translation.x = velocityTransform.x;
        odom_transform.transform.translation.y = 0.0;
        odom_transform.transform.translation.z = 0.5;
        odom_transform.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

        odomTransform_.sendTransform(odom_transform);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
