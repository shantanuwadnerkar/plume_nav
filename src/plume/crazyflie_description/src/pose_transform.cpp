#include <iostream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class VelocityTransform
{
    public:
        VelocityTransform();
        ~VelocityTransform();
        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
        
        double x_vel{0.0};
        double y_vel{0.0};
        double z_vel{0.0};
        double yaw_rate{0.0};
        double x{0.0};
        double y{0.0};
        double z{0.5};
        double yaw{0.0};
        ros::Time current_time = ros::Time::now();
        ros::Time last_time = ros::Time::now();

    private:
        double dx, dy, dtheta;
        double dt = 0.1;
        int first_time = 1;
};

VelocityTransform::VelocityTransform()
{}

VelocityTransform::~VelocityTransform()
{}

void VelocityTransform::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("OOP");
    // current_time = ros::Time::now();
    // if (first_time) 
    // {
    //     last_time = current_time;
    //     first_time = 0;
    // }
    x_vel = msg->linear.x;
    y_vel = msg->linear.y;
    z_vel = msg->linear.z;
    yaw_rate = msg->angular.z;

    // dt = (current_time-last_time).toSec();
    dx = (x_vel * cos(yaw) - y_vel * sin(yaw)) * dt;
    dy = (x_vel * sin(yaw) + y_vel * cos(yaw)) * dt;
    dtheta = yaw_rate * dt;

    x += dx;
    y += dy;
    yaw += dtheta;

    // last_time = current_time;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_transform");
    ros::NodeHandle n;

    VelocityTransform velocityTransform;

    tf::TransformBroadcaster mapToOdom_;
    tf::Transform map_transform_{ tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0) };

    tf::TransformBroadcaster odomTransform_;
    geometry_msgs::TransformStamped odom_transform;
    nav_msgs::Odometry odom;

    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1, &VelocityTransform::cmd_vel_callback, &velocityTransform);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("base_pose_ground_truth",10);

    ros::Rate rate(100);

    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_link";
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    while (ros::ok())
    {
        mapToOdom_.sendTransform(tf::StampedTransform(map_transform_, velocityTransform.current_time, "map", "odom"));        

        // ROS_INFO("%.2f, %.2f, %.2f", velocityTransform.x, velocityTransform.y, velocityTransform.z);
        
        odom_transform.header.stamp = velocityTransform.current_time;
        odom_transform.transform.translation.x = velocityTransform.x;
        odom_transform.transform.translation.y = velocityTransform.y;
        odom_transform.transform.translation.z = velocityTransform.z;
        odom_transform.transform.rotation = tf::createQuaternionMsgFromYaw(velocityTransform.yaw);

        odomTransform_.sendTransform(odom_transform);

        // Filling odometry
        // Setting position
        odom.header.stamp = velocityTransform.current_time;
        odom.pose.pose.position.x = velocityTransform.x;
        odom.pose.pose.position.y = velocityTransform.y;
        odom.pose.pose.position.z = velocityTransform.z;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(velocityTransform.yaw);

        // Setting velocity
        odom.twist.twist.linear.x = velocityTransform.x_vel;
        odom.twist.twist.linear.y = velocityTransform.y_vel;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.z = velocityTransform.yaw_rate;

        odom_pub.publish(odom);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
