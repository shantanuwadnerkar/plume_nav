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
        self.simulation_time = 1.0

        try:
            self.drone_x = rospy.get_param("/crazyflie_pose_transform/drone_spawn_x")
            self.drone_y = rospy.get_param("/crazyflie_pose_transform/drone_spawn_y")
            self.drone_z = rospy.get_param("/crazyflie_pose_transform/drone_spawn_z")
            self.drone_heading = rospy.get_param("/crazyflie_pose_transform/drone_spawn_heading")
        except KeyError:
            print("HELP!!!")

        self.concentration_curr = 0.0
        self.concentration_prev = 0.0

        self.wind_speed = 0.0
        self.wind_direction = 0.0

        self.max_source_prob_x = 0.0
        self.max_source_prob_y = 0.0
        self.max_source_prob_z = 0.0
        
        self._position_epsilon = 1e-4
        # Define some epsilon based on the sensor configuration
        # Random value. Change later
        self._concentration_epsilon = 1e-4

        # Define some lamba. Based on what?
        # Random value. Change later
        self._probability_threshold = 1e-4

        self._move_step = 0.5

        self.waypoint_client = actionlib.SimpleActionClient('waypoint', waypointAction)
        # self.waypoint_client.wait_for_server()
        self.goal = waypointGoal()

        self.waypoint_x_prev = 0.0
        self.waypoint_y_prev = 0.0
        self.waypoint_z_prev = 0.0
        self.waypoint_heading_prev = 0.5

        self.waypoint_x = 0.0
        self.waypoint_y = 0.0
        self.waypoint_z = 3.0
        self.waypoint_heading = 1.0
        # self.waypoint_heading = self.getInitialHeuristic()
        
        self.has_reached_waypoint = True
        # Start with some initial guess. How?


    def concentration_callback(self, concentration_reading):
        self.concentration_curr = concentration_reading.raw
        
        if self.has_reached_waypoint:
            # Substract the current sensor reading from the previous one
            concentration_change = self.concentration_curr - self.concentration_prev
            self.has_reached_waypoint = False
            rospy.loginfo(self.has_reached_waypoint)
            rospy.loginfo(concentration_change)
            if concentration_change >= self._concentration_epsilon:
                # If the concentration is higher than or equal to epsilon, continue in the same direction
                self.sendCurrentHeuristic()
            else:
                # else, find the probability that the current direction is right
                maintain_direction_prob = math.exp((concentration_change - self._concentration_epsilon)/self.simulation_time)
                
                if maintain_direction_prob > self._probability_threshold:
                    self.sendCurrentHeuristic()
                else:
                    self.sendNewHeuristic()


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
        self.max_source_prob_x = msg.x
        self.max_source_prob_y = msg.y
        self.max_source_prob_z = msg.z
        strr = self.max_source_prob_x, self.max_source_prob_y, self.max_source_prob_z
        # rospy.loginfo(strr)


    def isSource(self):
        # If the current cell is the source, stop
        if abs(self.max_source_prob_x - self.drone_x) < self._position_epsilon and abs(self.max_source_prob_y - self.drone_y) < self._position_epsilon:
            return True

        return False


    def getInitialHeuristic(self):
        self.waypoint_heading = math.atan2((self.max_source_prob_y - self.drone_y) / (self.max_source_prob_x - self.drone_x))


    def sendNewHeuristic(self):
        self.waypoint_heading = math.atan2((self.max_source_prob_y - self.drone_y) / (self.max_source_prob_x - self.drone_x))
        self.followDirection()


    def sendCurrentHeuristic(self):
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
        self.waypoint_x = self._move_step*math.cos(self.waypoint_heading) + self.waypoint_x_prev
        self.waypoint_y = self._move_step*math.sin(self.waypoint_heading)+ self.waypoint_y_prev
        self.sendWaypoint(self.waypoint_x,self.waypoint_y,self.waypoint_z)


    def sendWaypoint(self,x,y,z):
        goal = Point(x,y,z)
        self.waypoint_client.send_goal(goal, feedback_cb=self.actionFeedback)


    def actionFeedback(self):
        pass


    def raster_search(self):
        pass


    def raster_search_extended(self):
        pass


def testMetaheuristic():
    velocity_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    velocity_msg = Twist()
    velocity_msg.linear.x = -0.3
    velocity_publisher.publish(velocity_msg)


if __name__ == "__main__":
    try:
        rospy.init_node("heuristic")

        # Publish next waypoint in this node. This waypoint will be used by a waypoint manager to publish to cmd_vel
        # Change the topic name and topic type to something sensible
        waypoint_pub = rospy.Publisher("waypoint", Point, queue_size=10)
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
            if "--test" in sys.argv:
                testMetaheuristic()

    except rospy.ROSInterruptException:
        pass
