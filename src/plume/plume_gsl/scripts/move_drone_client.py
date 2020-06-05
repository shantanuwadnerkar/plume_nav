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

        rospy.Subscriber("base_pose_ground_truth", Odometry, callback=self.drone_position_callback)
        rospy.wait_for_message("base_pose_ground_truth", Odometry)

        self.has_reached_waypoint = True
        self._waypoint_resolution = 0.5
        self._position_epsilon = 1e-4

        self.waypoint_x_prev = self.drone_x
        self.waypoint_y_prev = self.drone_y
        self.waypoint_z_prev = self.drone_z
        self.waypoint_heading_prev = self.drone_heading
        self.waypoint_x = self.drone_x
        self.waypoint_y = self.drone_y
        self.waypoint_z = self.drone_z
        self.waypoint_heading = self.drone_heading


    def drone_position_callback(self, msg):
        rot_q = msg.pose.pose.orientation

        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z
        _, _, self.drone_heading = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


    def get_drone_position(self):
        return (self.drone_x, self.drone_y, self.drone_z)


    # def getWaypointFromPosition(self, position):
    #     pass


    def generateWaypoint(self, waypoint_heading):
        self.waypoint_x = self._waypoint_resolution * math.cos(waypoint_heading) + self.waypoint_x_prev
        self.waypoint_y = self._waypoint_resolution * math.sin(waypoint_heading) + self.waypoint_y_prev
        print("Waypoint", self.waypoint_x, self.waypoint_y)
        self.waypoint_x_prev = self.waypoint_x
        self.waypoint_y_prev = self.waypoint_y
        return (self.waypoint_x, self.waypoint_y, self.waypoint_z)


    def sendWaypoint(self, waypoint, wait_for_result=True):
        # Send waypoint and set has_reached_waypoint to false. Set this back to true when the feedback from server comes true
        self.has_reached_waypoint = False
        self.waypointGoal = waypointGoal([Point(waypoint[0], waypoint[1], waypoint[2])])
        # print("sendWaypoint")
        self.waypoint_client.send_goal(self.waypointGoal, done_cb=self.actionDone)
        if wait_for_result:
            self.waypoint_client.wait_for_result()


    def actionDone(self, status, result):
        self.has_reached_waypoint = True
        print("reached")


if __name__ == "__main__":
    rospy.init_node("move_drone_client")
    mdc = MoveDroneClient()
    rospy.spin()
