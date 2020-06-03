#!/usr/bin/env python

import math
import sys

import rospy

import actionlib
from crazyflie_control.msg import waypointGoal, waypointAction, waypointResult, waypointFeedback
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from olfaction_msgs.msg import anemometer, gas_sensor
import time
from tf.transformations import euler_from_quaternion


class Metaheuristic:
    def __init__(self):
        # catch errors for simulation_time or initialise it to an appropriate value
        self.simulation_time = 1.0
        # drone_spawn_heading is not yet added to rosparam
        try:
            self.drone_x = rospy.get_param("/crazyflie_pose_transform/drone_spawn_x")
            self.drone_y = rospy.get_param("/crazyflie_pose_transform/drone_spawn_y")
            self.drone_z = rospy.get_param("/crazyflie_pose_transform/drone_spawn_z")
            # self.drone_heading = rospy.get_param("/crazyflie_pose_transform/drone_spawn_heading")
        except KeyError:
            raise rospy.ROSInitException

        self.concentration_curr = 0.0
        self.concentration_prev = 0.0

        self.wind_speed = 0.0
        self.wind_direction = 0.0

        # This first initial readings from the sensor should be ignored as
        # they are very variant and the plume might not have reached the drone
        # in certain conditions. Assign the max probability of source as the
        # position of drone.
        self.max_source_prob_x = self.drone_x
        self.max_source_prob_y = self.drone_y
        self.max_source_prob_z = self.drone_z

        self._position_epsilon = 1e-4
        # Define some epsilon based on the sensor configuration
        # Random value. Change later
        self._concentration_epsilon = 1e-1

        # Define some lamba. Based on what?
        # Random value. Change later
        self._probability_threshold = 1e-4

        self.has_reached_waypoint = True
        self._move_step = 0.5
        self.skip_max_source_probability_msg = 0
        self.skip_max_source_probability_msg_count = 0

        self.waypoint_client = actionlib.SimpleActionClient('waypoint', waypointAction)
        self.waypoint_client.wait_for_server()
        self.goal = waypointGoal()

        # Previous waypoint would be the place where the drone is currently at
        self.waypoint_x_prev = self.drone_x
        self.waypoint_y_prev = self.drone_y
        self.waypoint_z_prev = self.drone_z
        self.waypoint_heading_prev = self.getNewWaypointHeading()

        self.waypoint_x = 0.0
        self.waypoint_y = 0.0
        self.waypoint_z = 3.0
        self.waypoint_heading = self.getNewWaypointHeading()
        
        
        # Start with some initial guess. How?


    def concentration_callback(self, concentration_reading):
        try:
            self.concentration_curr = concentration_reading.raw
        except rospy.ROSInterruptException:
            raise rospy.ROSInterruptException
        rospy.loginfo("===================================================1")
        if self.has_reached_waypoint:
            # Substract the current sensor reading from the previous one
            concentration_change = self.concentration_curr - self.concentration_prev
            rospy.loginfo("===================================================2")
            rospy.loginfo(self.has_reached_waypoint)
            print("Concntration Change", concentration_change)
            if concentration_change >= self._concentration_epsilon:
                rospy.loginfo("first if")
                # If the concentration is higher than or equal to epsilon, continue in the same direction
                self.sendCurrentHeuristic()
            else:
                # else, find the probability that the current direction is right
                maintain_direction_prob = math.exp((concentration_change - self._concentration_epsilon)/self.simulation_time)
                
                if maintain_direction_prob > self._probability_threshold:
                    self.sendCurrentHeuristic()
                else:
                    self.sendNewHeuristic()

            self.waypoint_x_prev = self.waypoint_x
            self.waypoint_y_prev = self.waypoint_y
            self.waypoint_z_prev = self.waypoint_z


    def drone_position_callback(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        _, _, self.drone_heading = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        strr = self.drone_x, self.drone_y, self.drone_heading
        # rospy.loginfo(strr)


    def wind_callback(self, msg):
        self.wind_speed = msg.wind_speed
        self.wind_direction = msg.wind_direction


    def concentration_mean(self):
        pass


    def max_source_probability_callback(self, msg):
        self.skip_max_source_probability_msg_count += 1
        if self.skip_max_source_probability_msg_count > self.skip_max_source_probability_msg:
            self.max_source_prob_x = msg.x
            self.max_source_prob_y = msg.y
            self.max_source_prob_z = msg.z
            # strr = self.max_source_prob_x, self.max_source_prob_y, self.max_source_prob_z
            # rospy.loginfo(strr)


    def isSource(self):
        # If the current cell is the source, stop
        if abs(self.max_source_prob_x - self.drone_x) < self._position_epsilon and abs(self.max_source_prob_y - self.drone_y) < self._position_epsilon:
            return True
        return False


    def getInitialHeuristic(self):
        rospy.loginfo("getInitialHeuristic")
        self.waypoint_heading = self.getNewWaypointHeading()


    def getNewWaypointHeading(self):
        rospy.loginfo("getNewWaypointHeading")
        # If there is division by zero, return heading of zero angle
        try:
            heading = math.atan2((self.max_source_prob_y - self.drone_y) / (self.max_source_prob_x - self.drone_x))
        except ZeroDivisionError:
            heading = 0.0
        return heading


    def sendNewHeuristic(self):
        self.waypoint_heading = self.getNewWaypointHeading()
        self.followDirection()


    def sendCurrentHeuristic(self):
        rospy.loginfo("sendCurrentHeuristic")
        if self.isSource():
            # stop. source located
            pass
        else:
            # keep following the current direction
            self.followDirection()

            # and update simulation_time
            # Send a goal to waypoint action server 
        # Store the current sensor reading as the previous sensor reading


    def followDirection(self):
        rospy.loginfo("follow direction")
        self.waypoint_x = self._move_step * math.cos(self.waypoint_heading) + self.waypoint_x_prev
        self.waypoint_y = self._move_step * math.sin(self.waypoint_heading) + self.waypoint_y_prev
        print("Waypoint", self.waypoint_x, self.waypoint_y)
        self.sendWaypoint(self.waypoint_x,self.waypoint_y,self.waypoint_z)


    def sendWaypoint(self,x,y,z):
        # Send waypoint and set has_reached_waypoint to false. Set this back to true when the feedback from server comes true
        self.has_reached_waypoint = False
        goal = waypointGoal([Point(x,y,z)])
        self.waypoint_client.send_goal(goal, feedback_cb=self.actionFeedback, done_cb=self.actionDone)
        # dur = rospy.Duration(secs=3)
        # rospy.sleep(dur)
        # self.has_reached_waypoint = True


    def actionDone(self, status, result):
        self.has_reached_waypoint = True


    def actionFeedback(self):
        pass


    def raster_search(self):
        pass


    def raster_search_extended(self):
        pass


if __name__ == "__main__":
    try:
        rospy.init_node("heuristic")

        # Publish next waypoint in this node. This waypoint will be used by a waypoint manager to publish to cmd_vel
        # Change the topic name and topic type to something sensible
        # waypoint_pub = rospy.Publisher("waypoint", Point, queue_size=10)
        print("creating obj")
        mh = Metaheuristic()
        print("created obj")
        # Current position and speed
        rospy.Subscriber("base_pose_ground_truth", Odometry, callback=mh.drone_position_callback)

        # subscribe to concentration_reading
        rospy.Subscriber("PID/Sensor_reading", gas_sensor, callback=mh.concentration_callback)

        # Subscribe to Anemometer - get wind data
        rospy.Subscriber("Anemometer/WindSensor_reading", anemometer, callback=mh.wind_callback)

        rospy.Subscriber("max_probability", Point, callback=mh.max_source_probability_callback)

        while not rospy.is_shutdown():
            try:
                continue
            except rospy.ROSInterruptException:
                break

    except rospy.ROSInterruptException:
        pass