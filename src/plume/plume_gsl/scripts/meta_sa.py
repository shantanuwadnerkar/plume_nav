import math
import sys

import rospy

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from olfaction_msgs.msg import anemometer, gas_sensor
import time
from tf.transformations import euler_from_quaternion


class Metaheuristic:
    def __init__(self):
        self.simulation_time = 0.0

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_heading = 0.0

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

        # Start with some initial guess. How?


    def concentration_callback(self, concentration_reading):
        self.concentration_curr = concentration_reading.raw

        # Substract the current sensor reading from the previous one
        concentration_change = self.concentration_curr - self.concentration_prev

        if concentration_change >= self._concentration_epsilon:
            # If the concentration is higher than or equal to epsilon, continue in the same direction
            self.keepDirection()
        else:
            # else, find the probability that the current direction is right
            maintain_direction_prob = math.exp((concentration_change - self._concentration_epsilon)/self.simulation_time)
            
            if maintain_direction_prob > self._probability_threshold:
                self.keepDirection()
            else:
                self.getNewHeuristic()
                pass


    def drone_position_callback(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.position.orientation
        _, _, self.drone_heading = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


    def wind_callback(self, msg):
        self.wind_speed = msg.wind_speed
        self.wind_direction = msg.wind_direction


    def concentration_mean(self):
        pass


    def max_source_probability_callback(self, msg):
        self.max_source_prob_x = msg.x
        self.max_source_prob_y = msg.y
        self.max_source_prob_z = msg.z


    def isSource(self):
        # If the current cell is the source, stop
        if abs(self.max_source_prob_x - self.drone_x) < self._position_epsilon and abs(self.max_source_prob_y - self.drone_y) < self._position_epsilon:
            return True

        return False


    def getNewHeuristic(self):
        pass


    def keepDirection(self):
        if self.isSource():
            # stop. source located
            pass
        else:
            # keep following the current direction
            # and update simulation_time
            pass
        # Store the current sensor reading as the previous sensor reading


    def raster_search(self):
        pass


    def raster_search_extended(self):
        pass


if __name__ == "__main__":
    rospy.init_node("heuristic")

    # Publish next waypoint in this node. This waypoint will be used by a waypoint manager to publish to cmd_vel
    # Change the topic name and topic type to something sensible
    waypoint_pub = rospy.Publisher("waypoint", Point, queue_size=10)

    mh = Metaheuristic()

    # Current position and speed
    rospy.Subscriber("/base_pose_ground_truth", Odometry, callback=mh.drone_position_callback)

    # subscribe to concentration_reading
    rospy.Subscriber("PID/Sensor_reading", gas_sensor, callback=mh.concentration_callback)

    # Subscribe to Anemometer - get wind data
    rospy.Subscriber("Anemometer/WindSensor_reading", anemometer, callback=mh.wind_callback)

    rospy.Subscriber("max_probability", Point, callback=mh.max_source_probability_callback)
    
    rospy.spin()
