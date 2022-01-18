#!/usr/bin/env python
import rospy
import math
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped

def callback(data):
    pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    for i in range(10):
        empty_msg = GoalID()
        pub.publish(empty_msg)
        rate.sleep()
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("move_base_simple/goal", PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

