#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from nav_msgs.msg import OccupancyGrid



#try:
#	add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
#	resp1 = add_two_ints(x, y)
#	print(resp1)
#except rospy.ServiceException as e:
#	print("Service call failed: %s"%e)
rospy.init_node('test_node', anonymous=True)
msg = rospy.wait_for_message("/map", OccupancyGrid)
print(msg)
