#include <iostream>
#include <string>
#include <cstdlib>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class VelocityTransform
{
    public:
        VelocityTransform();
        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
        
        double x_vel;
        double y_vel;
        double z_vel;
        double yaw_rate;
        ros::Time message_time;

    private:
};

VelocityTransform::VelocityTransform()
{
    x_vel = 0.0;
    y_vel = 0.0;
    z_vel = 0.0;
    yaw_rate = 0.0;
    message_time = ros::Time::now() - ros::Duration(5);
}

void VelocityTransform::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("OOP");
    x_vel = msg->linear.x;
    y_vel = msg->linear.y;
    z_vel = msg->linear.z;
    yaw_rate = msg->angular.z;
    message_time = ros::Time::now();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_transform");
    ros::NodeHandle n;

    VelocityTransform vT;

    tf::TransformBroadcaster mapToOdom_;
    tf::Transform map_transform_{ tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0) };

    tf::TransformBroadcaster odomTransform_;
    geometry_msgs::TransformStamped odom_transform;
    nav_msgs::Odometry odom;

    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1, &VelocityTransform::cmd_vel_callback, &vT);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("base_pose_ground_truth",10);

    ros::Rate rate(100);

    double dt = 0.01;
    double timeout = 0.55;
    double dx, dy, dtheta;

    ros::Time current_time;
    ros::Time previous_time;

    // Following variables should be initialized from parameters
    double x{0.0};
    double y{0.0};
    double z{0.5};
    double yaw{0.0};

    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_link";
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    while (ros::ok())
    {
        current_time = ros::Time::now();

        mapToOdom_.sendTransform(tf::StampedTransform(map_transform_, current_time, "map", "odom"));        
        
        if (abs((current_time - vT.message_time).toSec()) >= timeout)
        {
            vT.x_vel = 0;
            vT.y_vel = 0;
            vT.yaw_rate = 0;
        }
        dx = (vT.x_vel * cos(yaw) - vT.y_vel * sin(yaw)) * dt;
        dy = (vT.x_vel * sin(yaw) + vT.y_vel * cos(yaw)) * dt;
        dtheta = vT.yaw_rate * dt;

        x += dx;
        y += dy;
        yaw += dtheta;        
        
        odom_transform.header.stamp = current_time;
        odom_transform.transform.translation.x = x;
        odom_transform.transform.translation.y = y;
        odom_transform.transform.translation.z = z;
        odom_transform.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);

        odomTransform_.sendTransform(odom_transform);

        // Filling odometry
        // Setting position
        odom.header.stamp = current_time;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = z;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        // Setting velocity
        odom.twist.twist.linear.x = vT.x_vel;
        odom.twist.twist.linear.y = vT.y_vel;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.z = vT.yaw_rate;

        odom_pub.publish(odom);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}