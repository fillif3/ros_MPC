#!/usr/bin/env python
import math

import rospy

import utils
from APF import navigate_APF
from fuzzyController import navigate_fuzzy


from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Pose
from sensor_msgs.msg import LaserScan

ALGORITHM=1
SCAN_HZ=5

destination=None
current_position=None
predicted_position=None
acml_update_flag=True

def change_destination(msg):
    global destination
    destination={
        'x':msg.position.x,
        'y': msg.position.y,
        'theta': utils.getHeading(msg.orientation)
    }
    #rospy.loginfo(destination)

def change_position(msg):
    global current_position,acml_update_flag
    current_position={
        'x':msg.pose.pose.position.x,
        'y': msg.pose.pose.position.y,
        'theta': utils.getHeading(msg.pose.pose.orientation)
    }
    acml_update_flag = True
    #rospy.loginfo(current_position)

def move(scan):
    global destination,current_position,acml_update_flag,predicted_position
    if destination is None or current_position is None:
        return None
    if acml_update_flag:
        predicted_position=current_position
        acml_update_flag=False
    #rospy.loginfo('Positions predicted/current/desitnation')
    #rospy.loginfo(predicted_position)
    #rospy.loginfo(current_position)
    #rospy.loginfo(destination)
    if ALGORITHM==0:
        v,w = navigate_APF(predicted_position,scan,destination)
    elif ALGORITHM==1:
        v,w = navigate_fuzzy(predicted_position,scan,destination)
    else:
        raise ValueError
    send_velocity(v,w)
    predicted_position = robot_motion_model(predicted_position,v,w,1/SCAN_HZ)
    
def robot_motion_model(state,v,w,timeStep):
    sign = lambda x: -1 if x < 0 else (1 if x > 0 else 0)
    if w==0:
        state['x']=state[0]+v*math.cos(state['theta'])*timeStep
        state['y']=state['y']+v*math.sin(state['theta'])*timeStep
    elif v==0:
        state['theta']=state['theta']+w*timeStep
    elif sign(w)*sign(v)>0:
        beta = state['theta']+math.pi/2
        R = v/w
        circleX = state['x']+math.cos(beta)*R
        circleY = state['y']+math.sin(beta)*R
        delta_theta= w*timeStep
        beta=beta+delta_theta
        state['theta']=state['theta']+delta_theta
        state['x']=circleX-math.cos(beta)*R
        state['y']=circleY-math.sin(beta)*R
    else:
        beta = state['theta']-math.pi/2
        R = v/w
        circleX = state['x']+math.cos(beta)*R
        circleY = state['y']+math.sin(beta)*R
        delta_theta= w*timeStep
        beta=beta-delta_theta
        state['theta']=state['theta']+delta_theta
        state['x']=circleX-math.cos(beta)*R
        state['y']=circleY-math.sin(beta)*R
    return state



def send_velocity(v,w):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    base_data = Twist()
    base_data.linear.x = v
    base_data.angular.z = w

    pub.publish(base_data)


def listener():
    global pub
    global pub2
    rospy.init_node('navigator', anonymous=True)
    rospy.Subscriber('destination', Pose, change_destination)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, change_position)
    rospy.Subscriber('scan', LaserScan, move)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':

    listener()

