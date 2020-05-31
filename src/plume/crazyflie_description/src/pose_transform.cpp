#include <iostream>
#include <string>
#include <cstdlib>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

# define PI 3.14159265358979323846

void normalize_angle(double& angle)
{
    while (angle <= -PI) angle += 2*PI;
    while (angle > PI) angle -= 2*PI;
}

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
    ros::init(argc, argv, "crazyflie_pose_transform");
    ros::NodeHandle nh;

    VelocityTransform vT;

    double drone_spawn_x{0.0};
    double drone_spawn_y{0.0};
    double drone_spawn_z{0.0};
    double drone_spawn_yaw{0.0};
    
    nh.param<double>("/crazyflie_pose_transform/drone_spawn_x", drone_spawn_x, 3.0);
    nh.param<double>("/crazyflie_pose_transform/drone_spawn_y", drone_spawn_y, 10.0);
    nh.param<double>("/crazyflie_pose_transform/drone_spawn_z", drone_spawn_z, 0.5);
    nh.param<double>("/crazyflie_pose_transform/drone_spawn_yaw", drone_spawn_yaw, 0.0);
    
    // Odom frame should be placed at this location wrt map frame representing drone spawn location
    // Initilize drone using rosparameters declared in launch file
    // x=15.0, y=10.0, z=0.5, yaw=0.0. These coordinates spawn the drone  in the center of the plume 12 m away from the source.
    tf::TransformBroadcaster mapToOdom_;
    tf::Transform map_transform_{ tf::Quaternion(0.0, 0.0, 0.0, 1), tf::Vector3(0.0, 0.0, 0.0) };

    tf::TransformBroadcaster odomTransform_;
    geometry_msgs::TransformStamped odom_transform;
    nav_msgs::Odometry odom;

    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1, &VelocityTransform::cmd_vel_callback, &vT);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("base_pose_ground_truth", 10);

    ros::Rate rate(100);

    double dt = 0.01;
    double timeout = 0.55;
    double dx, dy, dtheta;

    ros::Time current_time;
    ros::Time previous_time;

    // Drone should be spawned at the odom frame origin
    double x{drone_spawn_x};
    double y{drone_spawn_y};
    double z{drone_spawn_z};
    double yaw{drone_spawn_yaw};
    
    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_link";
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    while (ros::ok())
    {
        current_time = ros::Time::now();

        // TODO: Need to make map-> odom a static transform
        mapToOdom_.sendTransform(tf::StampedTransform(map_transform_, current_time, "map", "odom"));        
        
        if (abs((current_time - vT.message_time).toSec()) >= timeout)
        {
            vT.x_vel = 0;
            vT.y_vel = 0;
            vT.yaw_rate = 0;
        }
        
        // Calculate position from cmd_vel
        dx = (vT.x_vel * cos(yaw) - vT.y_vel * sin(yaw)) * dt;
        dy = (vT.x_vel * sin(yaw) + vT.y_vel * cos(yaw)) * dt;
        dtheta = vT.yaw_rate * dt;

        // Update position and angle
        x += dx;
        y += dy;
        yaw += dtheta;        
        normalize_angle(yaw);

        // Transform drone in Rviz
        odom_transform.header.stamp = current_time;
        odom_transform.transform.translation.x = x;
        odom_transform.transform.translation.y = y;
        odom_transform.transform.translation.z = z;
        odom_transform.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);

        odomTransform_.sendTransform(odom_transform);

        // Topic for publishing drone position and velocity for other nodes to subscribe
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
