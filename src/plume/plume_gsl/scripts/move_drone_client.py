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

        self.position = Point()

        rospy.Subscriber("base_pose_ground_truth", Odometry, callback=self.drone_position_callback)
        rospy.wait_for_message("base_pose_ground_truth", Odometry)

        self.has_reached_waypoint = True
        self._waypoint_resolution = 0.5
        self._position_epsilon = 1e-4

        self.waypoint_prev = self.get_drone_position()
        self.waypoint = self.get_drone_position()
        self.waypoint_heading = self.drone_heading

        self.wayPoint = Point()
        self.wayPoint.x = self.wayPoint.y = None
        self.wayPoint_prev = self.position   


    def drone_position_callback(self, msg):
        rot_q = msg.pose.pose.orientation

        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z
        _, _, self.drone_heading = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        self.position.x = msg.pose.pose.position.x
        self.position.y = msg.pose.pose.position.y
        self.position.z = msg.pose.pose.position.z
        _, _, self.heading = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


    def get_drone_position(self):
        return (self.drone_x, self.drone_y, self.drone_z)


    # def getWaypointFromPosition(self, position):
    #     pass


    def generateWaypoint(self, waypoint_heading):
        waypoint_x = self._waypoint_resolution * math.cos(waypoint_heading) + self.waypoint_prev[0]
        waypoint_y = self._waypoint_resolution * math.sin(waypoint_heading) + self.waypoint_prev[1]
        waypoint_z = self.get_drone_position()[2]
        self.waypoint = (waypoint_x, waypoint_y, waypoint_z)

        return (self.waypoint[0], self.waypoint[1], self.waypoint[2])

        # self.wayPoint.x = self._waypoint_resolution * math.cos(waypoint_heading) + self.wayPoint_prev.x
        # self.wayPoint.y = self._waypoint_resolution * math.sin(waypoint_heading) + self.wayPoint_prev.y
        # self.wayPoint.z = self.position.z
        # return self.wayPoint


    def sendWaypoint(self, waypoint, wait_for_result=True):
        # Send waypoint and set has_reached_waypoint to false. Set this back to true when the feedback from server comes true
        self.has_reached_waypoint = False
        self.waypointGoal = waypointGoal([Point(waypoint[0], waypoint[1], waypoint[2])])

        self.waypoint_client.send_goal(self.waypointGoal, done_cb=self.actionDone)
        if wait_for_result:
            self.waypoint_client.wait_for_result()
        self.waypoint_prev = self.get_drone_position()

        # self.has_reached_waypoint = False
        # self.waypointGoal = waypointGoal([waypoint])

        # self.waypoint_client.send_goal(self.waypointGoal, done_cb=self.actionDone)
        # if wait_for_result:
        #     self.waypoint_client.wait_for_result()
        # self.waypoint_prev = self.position


    def followDirection(self, waypoint_heading, waypoint_res=None):
        if not waypoint_res:
            waypoint_res = self._waypoint_resolution
        
        rospy.loginfo("follow direction")
        self.wayPoint.x = waypoint_res * math.cos(waypoint_heading) + self.wayPoint_prev.x
        self.wayPoint.y = waypoint_res * math.sin(waypoint_heading) + self.wayPoint_prev.y
        print("Waypoint", self.wayPoint.x, self.wayPoint.y)

        # if not self.xbounds.min < self.waypoint_x < self.xbounds.max \
        #         or not self.ybounds.min < self.waypoint_y < self.ybounds.max:
        #     rospy.logwarn("Map boundary reached.")

        self.sendWayPoint()

    def sendWayPoint(self):
        self.has_reached_waypoint = False
        self.waypointGoal = waypointGoal([self.wayPoint])
        self.waypoint_client.send_goal(self.waypointGoal, done_cb=self.actionDone)
        self.wayPoint_prev = self.wayPoint


    def actionDone(self, status, result):
        self.has_reached_waypoint = True



if __name__ == "__main__":
    rospy.init_node("move_drone_client")
    mdc = MoveDroneClient()
    rospy.spin()
