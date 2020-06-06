#!/usr/bin/env python

import math
import sys
from collections import namedtuple, deque
import numpy as np
import random
from json import loads

import rospy

import actionlib
from crazyflie_control.msg import waypointGoal, waypointAction, waypointResult, waypointFeedback
from plume_gsl.msg import rasterScanAction, rasterScanGoal, rasterScanResult, rasterScanFeedback
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from olfaction_msgs.msg import anemometer, gas_sensor
import time
import tf
from tf.transformations import euler_from_quaternion


class Metaheuristic:
    def __init__(self):
        # drone_spawn_heading is not yet added to rosparam
        self.FOLLOW_WIND = 0
        self.ZIGZAG = 1
        self.METAHEURISTIC = 2

        try:
            self.drone_x = rospy.get_param("/crazyflie_pose_transform/drone_spawn_x")
            self.drone_y = rospy.get_param("/crazyflie_pose_transform/drone_spawn_y")
            self.drone_z = rospy.get_param("/crazyflie_pose_transform/drone_spawn_z")
            self.drone_heading = rospy.get_param("/crazyflie_pose_transform/drone_spawn_yaw")
            self.algorithm = rospy.get_param("~/Algorithm",1)
            xlims = rospy.get_param("xlims","[0,20]")
            ylims = rospy.get_param("ylims","[0,20]")
        except KeyError:
            raise rospy.ROSInitException
        
        Range = namedtuple("Range", "min max")

        xlims = loads(xlims)
        ylims = loads(ylims)
        
        self.xbounds = Range(xlims[0], xlims[1])
        self.ybounds = Range(ylims[0], ylims[1])
        
        # Waypoint resolution parameters        
        self.waypoint_res = 0.5  

        # The following are to change the waypoint_resolution based on the concentration      
        self.res_range = Range(1, 2.5)
        self.conc_range = Range(10.0, 200.0)
        self.calcWaypointSlopeIntercept()

        self.waypoint_client = actionlib.SimpleActionClient('waypoint', waypointAction)
        self.waypoint_client.wait_for_server()
        self.waypointGoal = waypointGoal()

        self.listener = tf.TransformListener()
        self.fixed_frame = rospy.get_param("fixed_frame", "map")
        self.anemo_frame = rospy.get_param("anemometer_frame","anemometer_frame")

        # Previous and current states
        self.waypoint_x_prev = self.drone_x
        self.waypoint_y_prev = self.drone_y
        self.waypoint_z_prev = self.drone_z
        self.waypoint_heading_prev = self.drone_heading

        self.waypoint_x = None
        self.waypoint_y = None
        self.waypoint_z = None
        self.waypoint_heading = None

        # Temperature parameter
        self.Temp = 100.0
        self.delta_Temp = 2.0

        # Zig-zag angle
        self.alpha = math.radians(35)

        self.source_reached = False        

        self._position_epsilon = 1e-4
        # Define some epsilon based on the sensor configuration
        # Random value. Change later
        self._conc_grad_epsilon = 0

        # Minimum concentration to detect
        self._conc_epsilon = 10

        # Define some lamba. Based on what?
        # Random value. Change later
        self._probability_threshold = 1e-4

        self.maintain_dir_prob = 0.4

        self.has_reached_waypoint = True
        self.moving_to_source = False
        self.source_reached = False
                
        self.max_conc_at = Point()
        self.max_conc_val = None
        self.concentration_hist = []

        self.raster_scan_complete = True

        self.wind_hist = deque([])
        self.len_wind_hist = 15

    def actionDone(self, status, result):
        self.has_reached_waypoint = True
        
        if self.moving_to_source == True:
            self.source_reached = True

    def calcWaypointSlopeIntercept(self):
        self.waypoint_slope = (self.res_range.max - self.res_range.min)/(1/self.conc_range.min - 1/self.conc_range.max)
        self.waypoint_intercept = self.res_range.min - self.waypoint_slope*(1/self.conc_range.max)

    def callRasterScan(self):
        # Send raster scan goal
        self.raster_scan_complete = False
        pass
    
    def changeTemperature(self):
        # Change temperature
            self.Temp -= self.delta_Temp
            if self.Temp <= 0:
                rospy.logerr("Temperature has reduced below Zero")
    
    def checkGradient(self):
        L = len(self.concentration_hist)
        if L < 2:
            rospy.logerr("Concentration data not long enough")
        gradient = np.mean(self.concentration_hist[L/2:]) - np.mean(self.concentration_hist[:L/2])
        return gradient

    def dronePositionCallback(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        _, _, self.drone_heading = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
    def followDirection(self):
        rospy.loginfo("follow direction")
        self.waypoint_x = self.waypoint_res * math.cos(self.waypoint_heading) + self.waypoint_x_prev
        self.waypoint_y = self.waypoint_res * math.sin(self.waypoint_heading) + self.waypoint_y_prev
        print("Waypoint", self.waypoint_x, self.waypoint_y)

        if not self.xbounds.min < self.waypoint_x < self.xbounds.max \
                or not self.ybounds.min < self.waypoint_y < self.ybounds.max:
            rospy.logwarn("Map boundary reached. Max concentration found at (%f,%f)",self.max_conc_at.x, self.max_conc_at.y)

            self.moveToSource()        

        self.sendWaypoint(self.waypoint_x,self.waypoint_y,self.waypoint_z)

    def getInitialHeuristic(self):
        if len(self.wind_hist) == self.len_wind_hist:
            if self.algorithm == self.FOLLOW_WIND:
                self.waypoint_heading = math.pi + np.mean(self.wind_hist)
            
            elif self.algorithm == self.ZIGZAG:
                if random.random() >= 0.5:
                    self.alpha *= -1            
                
                self.waypoint_heading = math.pi + np.mean(self.wind_hist) + self.alpha
            return True
        else:
            return False

    def getNewHeuristic(self):
        if self.algorithm == self.FOLLOW_WIND:
            # Choose a random perpendicular-to-wind direction
            if random.random() >= 0.5:
                dir = 1
            else:
                dir = -1
            self.waypoint_heading = dir*math.pi/2 + np.mean(self.wind_hist)
        
        elif self.algorithm == self.ZIGZAG:
            self.alpha *= -1
            self.waypoint_heading = math.pi + np.mean(self.wind_hist) + self.alpha        

    def getNormalHeuristic(self):
        if self.algorithm == self.FOLLOW_WIND:
            self.waypoint_heading = math.pi + np.mean(self.wind_hist)
        elif self.algorithm == self.ZIGZAG:
            pass

    def maxSourceProbabilityCallback(self, msg):
        self.max_source_prob_x = msg.x
        self.max_source_prob_y = msg.y
        self.max_source_prob_z = msg.z

    def moveToSource(self):
        
        if self.algorithm == self.FOLLOW_WIND:
                self.moving_to_source = True

        if self.algorithm == self.ZIGZAG:
            # Should implement raster scan here before FOLLOW_WIND
            self.callRasterScan()
            if self.raster_scan_complete:
                self.algorithm = self.FOLLOW_WIND
        
        self.waypoint_x, self.waypoint_y = self.max_conc_at.x, self.max_conc_at.y


    def normalizing_angle(self, angle):
        while angle <= -math.pi:
            angle += 2*math.pi

        while angle > math.pi:
            angle -= 2*math.pi

        return angle

    def sendWaypoint(self,x,y,z):
        # Send waypoint and set has_reached_waypoint to false. Set this back to true when the feedback from server comes true
        self.has_reached_waypoint = False
        self.waypointGoal = waypointGoal([Point(x,y,z)])
        self.waypoint_client.send_goal(self.waypointGoal, done_cb=self.actionDone)

    def waypointResCalc(self):
        try:
            concentration = self.concentration_hist[-1]
            if concentration > self._conc_epsilon:
                self.waypoint_res = max(min(self.waypoint_slope/concentration + self.waypoint_intercept,
                                        self.res_range.max),self.res_range.min)
            else:
                self.waypoint_res = self.res_range.max

        except IndexError:
            self.waypoint_res = self.res_range.max
    
    def windCallback(self, msg):        
        self.listener.waitForTransform(self.anemo_frame,self.fixed_frame, rospy.Time(), rospy.Duration(5.0))
            
        # Orientation of sensor with respect to world frame
        _,rot = self.listener.lookupTransform(self.fixed_frame, self.anemo_frame, rospy.Time(0))
        _,_,yaw = euler_from_quaternion(rot)
        wind_dir = yaw + msg.wind_direction - math.pi

        # Normalizing angle
        wind_dir = self.normalizing_angle(wind_dir)
        self.wind_hist.append(wind_dir)

        # Delete old wind data
        if len(self.wind_hist) > self.len_wind_hist:
            self.wind_hist.popleft()

    def concentrationCallback(self, msg):        
        if not self.raster_scan_complete:
            # Check status of action
            # self.raster_scan_complete = 
            if self.raster_scan_complete:
                self.concentration_hist.append(msg.raw)

                # Compare maximum concentration

                self.getNormalHeuristic()
                self.waypointResCalc()
                self.followDirection()
                
                self.concentration_hist = []
            
            return

        if self.source_reached:
            rospy.loginfo_once("Source declared at (%f,%f)",self.drone_x, self.drone_y)
            return

        if not self.waypoint_x:
            if self.algorithm == self.METAHEURISTIC:
                self.callRasterScan()
                return

            elif self.getInitialHeuristic():
                self.waypointResCalc()
                self.followDirection()
        
        concentration = msg.raw

        # Record of maximum concentration
        if concentration > self.max_conc_val:
            self.max_conc_val = concentration
            self.max_conc_at.x, self.max_conc_at.y = self.drone_x, self.drone_y
        
        self.concentration_hist.append(concentration)

        elif self.has_reached_waypoint and not self.source_reached:
            
            rospy.loginfo("Length of concentration data: %d"%len(self.concentration_hist))

            self.waypoint_x_prev = self.waypoint_x
            self.waypoint_y_prev = self.waypoint_y
            self.waypoint_z_prev = self.waypoint_z

            gradient = self.checkGradient()

            if gradient > self._conc_grad_epsilon:
                self.getNormalHeuristic()
                self.waypointResCalc()
                self.followDirection()

                # self.changeTemperature()
                
            else:
                # maintain_dir_prob = math.exp((gradient - self._conc_grad_epsilon)/self.Temp)

                # If non-increasing gradient and low concentration
                if self.concentration_hist[-1] < self._conc_epsilon:
                    rospy.loginfo("Concentration too low. Getting new heuristic")
                    self.getNewHeuristic()
                    self.waypointResCalc()
                    self.followDirection()

                elif self.maintain_dir_prob > random.random():
                    rospy.loginfo("Maintain direction prob")
                    self.getNormalHeuristic()
                    self.waypointResCalc()
                    self.followDirection()
                    # self.changeTemperature()
                else:
                    rospy.loginfo("Getting new heuristic")
                    self.getNewHeuristic()
                    self.waypointResCalc()
                    self.followDirection()
                    # Calculate new direction
            
            self.concentration_hist = []


if __name__ == "__main__":
    try:
        rospy.init_node("metaheuristic")

        mh = Metaheuristic()

        # Current position and speed
        rospy.Subscriber("base_pose_ground_truth", Odometry, callback=mh.dronePositionCallback)
        rospy.wait_for_message("base_pose_ground_truth", Odometry)

        # Subscribe to Anemometer - get wind data
        rospy.Subscriber("Anemometer/WindSensor_reading", anemometer, callback=mh.windCallback)
        rospy.wait_for_message("Anemometer/WindSensor_reading", anemometer)

        rospy.Subscriber("max_probability", Point, callback=mh.maxSourceProbabilityCallback)
        rospy.wait_for_message("max_probability", Point)

        # subscribe to concentration_reading
        rospy.Subscriber("PID/Sensor_reading", gas_sensor, callback=mh.concentrationCallback)

        # Implement subscriber to subscribe to a range of points corresponding to high probability
        # rospy.Subscriber("some-topic", some_type,mh.some_callback)

        while not rospy.is_shutdown():
            try:
                continue
            except rospy.ROSInterruptException:
                break

    except rospy.ROSInterruptException:
        pass