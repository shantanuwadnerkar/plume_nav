#! /usr/bin/env python

import math
import sys
import time

import actionlib
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import rospy
from tf.transformations import euler_from_quaternion

from move_robot.msg import waypointAction, waypointGoal, waypointResult, waypointFeedback
        

def handle_shutdown():
    print("[COMPLETED] Reached. Shutting down")


class MoveDrone:
    def __init__(self):
        self.drone_curr_x = 0.0
        self.drone_curr_y = 0.0
        self.drone_curr_heading = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0

        self._position_epsilon = 1e-2

        self.actionlib_server = actionlib.SimpleActionServer('waypoint_heuristic', waypointAction, self.waypoint_callback, False)
        self.actionlib_server.start()


    def angular_difference(self, a, b):
        normalized_angle = (a - b) % (2 * math.pi)
        difference = min(2 * math.pi - normalized_angle, normalized_angle)
        return difference


    def compute_velocity_to_goal(self):
        # goal_distance = math.sqrt((self.drone_curr_x - self.goal_x) ** 2 + (self.drone_curr_y - self.goal_y) ** 2)
        # if self.has_reached_goal():
        #     angle_to_goal = math.atan2((goal_y - y), (goal_x - x))
        #     if abs(angle_to_goal - theta) > self._position_epsilon:
        #         vel_msg.linear.x = 0
        #         vel_msg.angular.z = 2 * self.angular_difference(theta, angle_to_goal)
        #     else:
        #         vel_msg.linear.x = 0.5 * goal_distance
        #         vel_msg.angular.z = 0
        #     return False
        # else:
        #     vel_msg.linear.x = 0
        #     vel_msg.angular.z = 0
        #     return True
        pass


    def has_reached_goal(self):
        # Use the below formula or do it using goal_distance. Decide how.
        if (abs(self.goal_x - self.drone_curr_x) < self._position_epsilon) and (abs(self.goal_y - self.drone_curr_y) < self._position_epsilon):
            return True
        else:
            return False



    def waypoint_callback(self, goal):
        # IMP!!!!!!!!!!!!!!!
        # Assign goal here from goal msg
        self.goal_x = 0.0
        self.goal_y = 0.0

        # handle failure to reach goal and/or preemption. Also, send feedback. Do this using has_reached_goal()

    def ground_truth_callback(self, msg):
        self.drone_curr_x = msg.pose.pose.position.x
        self.drone_curr_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.position.orientation
        _, _, self.drone_curr_heading = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


    def publish_cmd_vel(self):
        pass


if __name__ == "__main__":
    rospy.init_node("move_drone")
    rospy.on_shutdown(handle_shutdown)

    moveDrone = MoveDrone()

    sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, moveDrone.ground_truth_callback)
    
    rospy.spin()
