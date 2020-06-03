#! /usr/bin/env python

import math
import time

import actionlib
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import rospy
from tf.transformations import euler_from_quaternion

from crazyflie_control.msg import waypointAction, waypointGoal, waypointResult, waypointFeedback

x = 0; y = 0; theta = 0
vel_msg = Twist()
i = 0
t = time.time()

def sub_callback(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch,theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def angdiff(a, b):

    direction = (a-b)/abs(a-b)
    normdeg = (a-b)%(2*math.pi)
    diff = min(2*math.pi-normdeg, normdeg)
    return diff

def moving_controller(goal_x, goal_y):
        
        dist2goal = math.sqrt((x - goal_x)**2 + (y - goal_y)**2)

        if dist2goal > 0.05:

            angle_to_goal = math.atan2((goal_y - y), (goal_x - x))
            
            if abs(angle_to_goal - theta) > 0.15:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 2 * angdiff(theta, angle_to_goal)
            else:
                vel_msg.linear.x = 0.5 * dist2goal
                vel_msg.angular.z = 0
            return False
        
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            return True
        

def action_callback(goal):
    global i
    result = waypointResult()
    feedback = waypointFeedback()
    while i < len(goal.points):
        goal_x = goal.points[i].x
        goal_y = goal.points[i].y
        reached_checkpt = moving_controller(goal_x,goal_y)
        pub.publish(vel_msg)
        r.sleep()
        if reached_checkpt:
            i += 1
            feedback.num_completed = i
            feedback.total_points = len(goal.points)
            server.publish_feedback(feedback)
            if i < len(goal.points):
                reached_checkpt = False

    result.succ_msg = "Reached"
    server.set_succeeded(result)

    

rospy.init_node("action_server")
r = rospy.Rate(10)
sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, sub_callback)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
server = actionlib.SimpleActionServer('waypoints', waypointAction, action_callback, False)
server.start()
rospy.spin()