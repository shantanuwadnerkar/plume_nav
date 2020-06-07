#!/usr/bin/env python

import math

import rospy

import actionlib
from crazyflie_control.msg import waypointGoal, waypointAction, waypointResult, waypointFeedback
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MoveDroneClient:
    def __init__(self):
        self.waypoint_client = actionlib.SimpleActionClient('waypoint', waypointAction)       
        self.waypoint_client.wait_for_server()
        self.waypointGoal = waypointGoal()
        self.drone_position = Point()

        rospy.Subscriber("base_pose_ground_truth", Odometry, callback=self.drone_position_callback)
        rospy.wait_for_message("base_pose_ground_truth", Odometry)

        self.has_reached_waypoint = True
        self._waypoint_resolution = 0.5
        self._position_epsilon = 1e-4

        self.waypoint_prev = self.drone_position
        self.waypoint = self.drone_position
        self.waypoint_heading = self.drone_heading

    def drone_position_callback(self, msg):
        rot_q = msg.pose.pose.orientation

        self.drone_position.x = msg.pose.pose.position.x
        self.drone_position.y = msg.pose.pose.position.y
        self.drone_position.z = msg.pose.pose.position.z
        _, _, self.drone_heading = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def followDirection(self, waypoint_heading, waypoint_res=None):

        if not waypoint_res:
            waypoint_res = self._waypoint_resolution

        self.waypoint.x = waypoint_res * math.cos(waypoint_heading) + self.waypoint_prev.x
        self.waypoint.y = waypoint_res * math.sin(waypoint_heading) + self.waypoint_prev.y
        self.waypoint.z = self.drone_position.z
        
        self.sendWaypoint()

    def sendWaypoint(self):
        # Send waypoint and set has_reached_waypoint to false. Set this back to true when the feedback from server comes true
        self.has_reached_waypoint = False
        self.waypointGoal = waypointGoal([self.waypoint])
        self.waypoint_client.send_goal(self.waypointGoal, done_cb=self.actionDone)
        self.waypoint_prev = self.waypoint

    def actionDone(self, status, result):
        self.has_reached_waypoint = True



if __name__ == "__main__":
    rospy.init_node("move_drone_client")
    mdc = MoveDroneClient()
    rospy.spin()
