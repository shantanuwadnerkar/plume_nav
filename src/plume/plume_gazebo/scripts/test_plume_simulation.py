#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState


class TestPlumeSimulation:
    def __init__(self):
        self.filament_count = 0
        self.initial_wait = rospy.Rate(1)
        self.pose_per_iteration = rospy.Rate(1)

        self.state_service_handle = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)

    def countFilamentModels(self, msg):
        self.filament_count = len(msg.name) - 1

    def setFilamentState(self):
        self.initial_wait.sleep()
        # state_waypoint = [1.0]
        for i in range(100):
            for j in range(self.filament_count):
                state_msg = ModelState()
                state_msg.model_name = "filament_" + str(j)
                state_msg.pose.position.x = i*0.1
                state_msg.pose.position.y = (j - self.filament_count/2)*0.1
                state_msg.pose.position.z = 0.0
                state_msg.pose.orientation.x = 0
                state_msg.pose.orientation.y = 0
                state_msg.pose.orientation.z = 0
                state_msg.pose.orientation.w = 1.0

                try:
                    response = self.state_service_handle(state_msg)
                except rospy.ServiceException as e:
                    rospy.logwarn("Service call failed: %s" % e)

            self.pose_per_iteration.sleep()


def main():
    rospy.init_node("test_plume_simulation")

    rospy.wait_for_service("/gazebo/set_model_state")

    tps = TestPlumeSimulation()

    rospy.Subscriber("/gazebo/model_states", ModelStates,
                     tps.countFilamentModels)

    tps.setFilamentState()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Testing interrupted!\nShutting down...")
