import math
from math import pi

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


# FOR REACHING GOAL

error_linear = ctrl.Antecedent(np.arange(0, 20, 0.1), 'error_linear')
error_linear["QZ"]=fuzz.trimf(error_linear.universe,[0,0,0.7])
error_linear["VC"]=fuzz.trimf(error_linear.universe,[0,0.7,1.1])
error_linear["C"]=fuzz.trimf(error_linear.universe,[0.7,1.1,1.5])
error_linear["F"]=fuzz.trimf(error_linear.universe,[1.1,1.5,2])
error_linear["VF"]=fuzz.trapmf(error_linear.universe,[1.5,2,20,20])

error_angular = ctrl.Antecedent(np.arange(-pi, pi, 0.01), 'error_angular')
error_angular["NB"]=fuzz.trapmf(error_angular.universe,[-pi,-pi,-pi/3,-pi/10])
error_angular["NS"]=fuzz.trimf(error_angular.universe,[-pi/3,-pi/10,0])
error_angular["QZ"]=fuzz.trimf(error_angular.universe,[-pi/10,0,pi/10])
error_angular["PS"]=fuzz.trimf(error_angular.universe,[0,pi/10,pi/3])
error_angular["PB"]=fuzz.trapmf(error_angular.universe,[pi/10,pi/3,pi,pi])


velocity_linear = ctrl.Consequent(np.arange(0, 1, 0.01), 'velocity_linear')
velocity_linear["QZ"]=fuzz.trimf(velocity_linear.universe,[0,0,0.2])
velocity_linear["VS"]=fuzz.trimf(velocity_linear.universe,[0,0.2,0.35])
velocity_linear["S"]=fuzz.trimf(velocity_linear.universe,[0.20,0.35,0.70])
velocity_linear["F"]=fuzz.trimf(velocity_linear.universe,[0.35,0.70,0.90])
velocity_linear["VF"]=fuzz.trapmf(velocity_linear.universe,[0.7,0.90,1,1])

velocity_linear_rule_qz = ctrl.Rule(antecedent=(error_angular['NB'] | error_angular['PB'] | error_linear['QZ']), consequent=velocity_linear['QZ'])
velocity_linear_rule_vs = ctrl.Rule(antecedent=error_linear['VC'], consequent=velocity_linear['VS'])
velocity_linear_rule_s = ctrl.Rule(antecedent=error_linear['C'], consequent=velocity_linear['S'])
velocity_linear_rule_f = ctrl.Rule(antecedent=error_linear['F'], consequent=velocity_linear['F'])
velocity_linear_rule_vf = ctrl.Rule(antecedent=error_linear['VF'], consequent=velocity_linear['VF'])

fc_linear_ctrl = ctrl.ControlSystem([velocity_linear_rule_qz, velocity_linear_rule_vs,
                                   velocity_linear_rule_s,velocity_linear_rule_f,velocity_linear_rule_vf])
fc_linear = ctrl.ControlSystemSimulation(fc_linear_ctrl)



velocity_angular = ctrl.Consequent(np.arange(-1, 1, 0.01), 'velocity_angular')
velocity_angular["NB"]=fuzz.trapmf(velocity_angular.universe,[-2*pi,-2*pi,-pi/2,-pi/10])
velocity_angular["NS"]=fuzz.trimf(velocity_angular.universe,[-2*pi,-pi/10,0])
velocity_angular["QZ"]=fuzz.trimf(velocity_angular.universe,[-pi/10,0,pi/10])
velocity_angular["PS"]=fuzz.trimf(velocity_angular.universe,[0,pi/10,2*pi])
velocity_angular["PB"]=fuzz.trapmf(velocity_angular.universe,[pi/10,pi/2,pi*2,pi*2])

velocity_angular_rule_NB = ctrl.Rule(error_angular['NB'], velocity_angular['NB'])
velocity_angular_rule_NS = ctrl.Rule(error_angular['NS'], velocity_angular['NS'])
velocity_angular_rule_QZ = ctrl.Rule(error_angular['QZ'], velocity_angular['QZ'])
velocity_angular_rule_PS = ctrl.Rule(error_angular['PS'], velocity_angular['PS'])
velocity_angular_rule_PB = ctrl.Rule(error_angular['PB'], velocity_angular['PB'])

fc_angular_ctrl = ctrl.ControlSystem([velocity_angular_rule_NB, velocity_angular_rule_NS,
                                   velocity_angular_rule_QZ,velocity_angular_rule_PS,velocity_angular_rule_PB])
fc_angular = ctrl.ControlSystemSimulation(fc_angular_ctrl)

# FOR AVOIDING OBSTACLES

obstacle_distance_left = ctrl.Antecedent(np.arange(0, 2, 0.01), 'obstacle_distance_left')
obstacle_distance_left["VC"]=fuzz.trapmf(obstacle_distance_left.universe,[0,0,0.4,0.6])
obstacle_distance_left["C"]=fuzz.trimf(obstacle_distance_left.universe,[0.5,0.7,0.9])
obstacle_distance_left["F"]=fuzz.trapmf(obstacle_distance_left.universe,[0.8,1,2,2])

obstacle_distance_middle = ctrl.Antecedent(np.arange(0, 2, 0.01), 'obstacle_distance_middle')
obstacle_distance_middle["VC"]=fuzz.trapmf(obstacle_distance_middle.universe,[0,0,0.4,0.6])
obstacle_distance_middle["C"]=fuzz.trimf(obstacle_distance_middle.universe,[0.5,0.7,0.9])
obstacle_distance_middle["F"]=fuzz.trapmf(obstacle_distance_middle.universe,[0.8,1,2,2])

obstacle_distance_right = ctrl.Antecedent(np.arange(0, 2, 0.01), 'obstacle_distance_right')
obstacle_distance_right["VC"]=fuzz.trapmf(obstacle_distance_right.universe,[0,0,0.4,0.6])
obstacle_distance_right["C"]=fuzz.trimf(obstacle_distance_right.universe,[0.5,0.7,0.9])
obstacle_distance_right["F"]=fuzz.trapmf(obstacle_distance_right.universe,[0.8,1,2,2])

velocity_linear_obs = ctrl.Consequent(np.arange(0, 1, 0.01), 'velocity_linear_obs')
velocity_linear_obs["QZ"]=fuzz.trimf(velocity_linear_obs.universe,[0,0,0.3])
velocity_linear_obs["VS"]=fuzz.trimf(velocity_linear_obs.universe,[0,0.3,0.6])
velocity_linear_obs["S"]=fuzz.trapmf(velocity_linear_obs.universe,[0.30,0.6,1,1])

velocity_linear_obs_rule_qz = ctrl.Rule(antecedent=(obstacle_distance_left['VC'] | obstacle_distance_middle['VC'] |
                                    obstacle_distance_right['VC']), consequent=velocity_linear_obs['QZ'])
velocity_linear_obs_rule_vs = ctrl.Rule(
    (obstacle_distance_left['C'] & obstacle_distance_middle['C'] & obstacle_distance_right['C']) |
(obstacle_distance_left['C'] & obstacle_distance_middle['C'] & obstacle_distance_right['F']) |
(obstacle_distance_left['C'] & obstacle_distance_middle['F'] & obstacle_distance_right['C']) |
(obstacle_distance_left['C'] & obstacle_distance_middle['F'] & obstacle_distance_right['F']) |
(obstacle_distance_left['F'] & obstacle_distance_middle['C'] & obstacle_distance_right['C']) |
(obstacle_distance_left['F'] & obstacle_distance_middle['C'] & obstacle_distance_right['F']) |
(obstacle_distance_left['F'] & obstacle_distance_middle['F'] & obstacle_distance_right['C']),velocity_linear_obs['VS']
)
velocity_linear_obs_rule_s = ctrl.Rule(obstacle_distance_left['F'] & obstacle_distance_middle['F'] &
                                    obstacle_distance_right['F'], velocity_linear_obs['S'])

fc_linear_obs_ctrl = ctrl.ControlSystem([velocity_linear_obs_rule_qz, velocity_linear_obs_rule_vs,velocity_linear_obs_rule_s,])
fc_linear_obs = ctrl.ControlSystemSimulation(fc_linear_obs_ctrl)

velocity_angular_obs = ctrl.Consequent(np.arange(-3, 3, 0.01), 'velocity_angular_obs')
velocity_angular_obs["NVB"]=fuzz.trapmf(velocity_angular_obs.universe,[-3,-3,-2.5,-2])
velocity_angular_obs["NB"]=fuzz.trimf(velocity_angular_obs.universe,[-2.5,-2,-1])
velocity_angular_obs["NS"]=fuzz.trimf(velocity_angular_obs.universe,[-2,-1,-0.4])
velocity_angular_obs["QZ"]=fuzz.trimf(velocity_angular_obs.universe,[-0.7,0,0.7])
velocity_angular_obs["PS"]=fuzz.trimf(velocity_angular_obs.universe,[0.4,1,2])
velocity_angular_obs["PB"]=fuzz.trimf(velocity_angular_obs.universe,[1,2,2.5])
velocity_angular_obs["PVB"]=fuzz.trapmf(velocity_angular_obs.universe,[2,2.5,3,3])

velocity_angular_obs_rule_nvb = ctrl.Rule(
    (obstacle_distance_left['VC'] & obstacle_distance_middle['VC'] & obstacle_distance_right['VC']) |
(obstacle_distance_left['VC'] & obstacle_distance_middle['VC'] & obstacle_distance_right['C']) |
(obstacle_distance_left['VC'] & obstacle_distance_middle['VC'] & obstacle_distance_right['F']) |
(obstacle_distance_left['VC'] & obstacle_distance_middle['C'] & obstacle_distance_right['VC']) |
(obstacle_distance_left['VC'] & obstacle_distance_middle['F'] & obstacle_distance_right['F']) |
(obstacle_distance_left['C'] & obstacle_distance_middle['VC'] & obstacle_distance_right['C']) |
(obstacle_distance_left['C'] & obstacle_distance_middle['VC'] & obstacle_distance_right['F']) |
(obstacle_distance_left['F'] & obstacle_distance_middle['VC'] & obstacle_distance_right['F']) , velocity_angular_obs['NVB'])

velocity_angular_obs_rule_nb = ctrl.Rule(
    (obstacle_distance_left['VC'] & obstacle_distance_middle['C'] & obstacle_distance_right['C']) |
(obstacle_distance_left['VC'] & obstacle_distance_middle['C'] & obstacle_distance_right['F']) |
(obstacle_distance_left['C'] & obstacle_distance_middle['C'] & obstacle_distance_right['C']), velocity_angular_obs['NB'])

velocity_angular_obs_rule_ns = ctrl.Rule(
    (obstacle_distance_left['VC'] & obstacle_distance_middle['F'] & obstacle_distance_right['C']) |
(obstacle_distance_left['C'] & obstacle_distance_middle['C'] & obstacle_distance_right['F']) |
(obstacle_distance_left['C'] & obstacle_distance_middle['F'] & obstacle_distance_right['C']) |
(obstacle_distance_left['C'] & obstacle_distance_middle['F'] & obstacle_distance_right['F']) |
(obstacle_distance_left['F'] & obstacle_distance_middle['C'] & obstacle_distance_right['F']) |
(obstacle_distance_left['F'] & obstacle_distance_middle['F'] & obstacle_distance_right['F']) , velocity_angular_obs['NS'])

velocity_angular_obs_rule_qz=ctrl.Rule((obstacle_distance_left['VC'] & obstacle_distance_middle['F'] & obstacle_distance_right['VC']) , velocity_angular_obs['QZ'])

velocity_angular_obs_rule_ps = ctrl.Rule(
    (obstacle_distance_left['F'] & obstacle_distance_middle['F'] & obstacle_distance_right['C']) |
(obstacle_distance_left['F'] & obstacle_distance_middle['C'] & obstacle_distance_right['C']), velocity_angular_obs['PS'])

velocity_angular_obs_rule_pb=ctrl.Rule((obstacle_distance_left['C'] & obstacle_distance_middle['F'] & obstacle_distance_right['VC']) , velocity_angular_obs['PB'])

velocity_angular_obs_rule_pvb = ctrl.Rule(
    (obstacle_distance_left['C'] & obstacle_distance_middle['VC'] & obstacle_distance_right['VC']) |
(obstacle_distance_left['C'] & obstacle_distance_middle['C'] & obstacle_distance_right['VC']) |
(obstacle_distance_left['F'] & obstacle_distance_middle['VC'] & obstacle_distance_right['VC']) |
(obstacle_distance_left['F'] & obstacle_distance_middle['VC'] & obstacle_distance_right['C']) |
(obstacle_distance_left['F'] & obstacle_distance_middle['C'] & obstacle_distance_right['VC']) |
(obstacle_distance_left['F'] & obstacle_distance_middle['F'] & obstacle_distance_right['VC']) , velocity_angular_obs['PVB'])

fc_angular_obs_ctrl = ctrl.ControlSystem([velocity_angular_obs_rule_nvb, velocity_angular_obs_rule_nb,
                                   velocity_angular_obs_rule_ns,velocity_angular_obs_rule_qz,velocity_angular_obs_rule_ps,
                                                velocity_angular_obs_rule_pb,velocity_angular_obs_rule_pvb])
fc_angular_obs = ctrl.ControlSystemSimulation(fc_angular_obs_ctrl)

def navigate_fuzzy(estimated_pose,scan,destination):
    if obstacle_far(scan):
        v,w=fuzzy_control_destination(estimated_pose,destination)
    else:
        v,w=fuzzy_control_avoid_obstacle(scan)

    return v,w
# New Antecedent/Consequent objects hold universe variables and membership
# functions

def fuzzy_control_destination(estimated_pose,destination):
    global fc_linear,fc_angular
    diff_x = destination['x'] - estimated_pose['x']
    diff_y = destination['y'] - estimated_pose['y']
    angle =math.atan2(diff_y,diff_x)
    dist = math.sqrt(diff_x**2+diff_y**2)
    if dist>2:
        dist=2
    angle_error = angle-estimated_pose['theta']
    while angle_error>pi:
        angle_error=angle_error-2*pi
    while angle_error<-pi:
        angle_error=angle_error+2*pi
    fc_linear.input['error_linear']=dist
    fc_linear.input['error_angular']=angle_error
    print(fc_linear.input)
    fc_linear.compute()
    v = fc_linear.output['velocity_linear']
    fc_angular.input['error_angular'] = angle_error
    fc_angular.compute()
    w = fc_angular.output['velocity_angular']
    print(fc_linear.output)
    print(fc_angular.output)

    return v/10,w

def get_scans_groups(scan):
    scans_groups=[]
    angle = scan.angle_min + 10 * scan.angle_increment  # compute angle of first group
    amount_of_lasers = len(scan.ranges)
    amount_of_groups = int(amount_of_lasers / 10)  # we ahve ten times less groups than lasers
    for i in range(amount_of_groups):
        dist = 0
        number_of_active_lasers = 0
        for j in range(20):  # each group has 20 lasers
            if i * 10 + j < amount_of_lasers:  # condtion for last group
                if not math.isnan(scan.ranges[i * 10 + j]):  # some data from lasers are NaN
                    if scan.range_max > scan.ranges[
                        i * 10 + j] > scan.range_min:  # Range must between min and max range
                        dist = dist + scan.ranges[i * 10 + j]  # add laser data
                        number_of_active_lasers= number_of_active_lasers + 1
        if dist > 0:
            # if dist < ((negative_border * 20)+1.5): #f obstacle is close
            # dist = dist / 20 #divide sum
            dist = dist / number_of_active_lasers
            scans_groups.append({'angle':angle,"dist":dist})

        angle = angle + 10 * scan.angle_increment  # increament angle for next group
        if angle>pi:
            angle=angle-2*pi
    return scans_groups


def fuzzy_control_avoid_obstacle(scan):
    global fc_linear_obs, fc_angular_obs
    scans_groups = get_scans_groups(scan)
    left=2
    forward=2
    right=2
    for s in scans_groups:
        if (-pi*7/20)<s['angle']<(-pi*7/400):
            left = min(left,s['dist'])
        elif (-pi*7/400)<=s['angle']<=(pi*7/400):
            forward = min(forward, s['dist'])
        elif (pi * 7 / 400) < s['angle'] < (pi * 7 / 20):
            right = min(right, s['dist'])
    fc_linear_obs.input['obstacle_distance_left']=left
    fc_linear_obs.input['obstacle_distance_middle'] = forward
    fc_linear_obs.input['obstacle_distance_right'] = right
    print(fc_linear_obs.input)
    fc_linear_obs.compute()
    v=fc_linear_obs.output['velocity_linear_obs']
    fc_angular_obs.input['obstacle_distance_left'] = left
    fc_angular_obs.input['obstacle_distance_middle'] = forward
    fc_angular_obs.input['obstacle_distance_right'] = right

    fc_angular_obs.compute()
    w=fc_angular_obs.output['velocity_angular_obs']
    return v/10,w

def obstacle_far(scan):
    groups= get_scans_groups(scan)
    for g in groups:
        if g['dist']<0.4:
            return False
    return True


