#! /usr/bin/env python
import rospy
from gaden_player.srv import GasPosition

rospy.init_node("timing")
r = rospy.Rate(2)

rospy.wait_for_service("odor_value",rospy.Duration(10))
odor_req = rospy.ServiceProxy('odor_value', GasPosition)
a = rospy.Time.now()

while not rospy.is_shutdown():
    # print(rospy.Time.now())
    # print(rospy.Time(0).to_sec())
    rospy.loginfo("I am here")
    try:
        odor_res = odor_req(19,10, 3)
    except rospy.ServiceException, e:
        rospy.logerr("Odor service call failed %s"%e)

    if odor_res.gas_conc[0] > 0:
        rospy.logerr("%f",(rospy.Time.now()-a).to_sec())
    r.sleep()