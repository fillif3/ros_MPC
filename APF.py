import math

KP=1
KN=0.002
MAX_V=0.22
MAX_W=2.84
POSITIVE_BORDER=1
NEGATIVE_BORDER=0.6
ROBOT_RADIUS=0.075
PV=0.2
PW=0.4

def navigate_APF(estimated_pose,scan,destination):
    diff_x=destination['x']-estimated_pose['x']
    diff_y = destination['y'] - estimated_pose['y']
    force_positive_x, force_positive_y = positive_force(diff_x, diff_y)  # compute positive foces usuing parameters
    force_positive_x, force_positive_y = rotation(force_positive_x, force_positive_y, estimated_pose['theta'])
    force_negative_x, force_negative_y = negative_force(scan)
    force_robot_x = force_positive_x + force_negative_x  # add them
    force_robot_y = force_positive_y + force_negative_y
    v, w = controller(force_robot_x, force_robot_y)
    return v,w

def rotation(vx, vy, yaw): #Rotate vector about chosen angle
    v = vx * math.cos(yaw) + vy * math.sin(yaw)
    w = - vx * math.sin(yaw) + vy * math.cos(yaw)
    return v,w

def negative_force(scan):  # Compute negative force
    forcex = 0
    forcey = 0
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
            if dist < ((NEGATIVE_BORDER +ROBOT_RADIUS)  * number_of_active_lasers):
                # dist = dist / 20 #divide sum
                dist = dist / number_of_active_lasers
                if dist > ROBOT_RADIUS:  # robot is not a point so we need to substract a value if scan is lower, we set value to low number
                    dist = dist - ROBOT_RADIUS
                else:
                    dist = 0.0001
                force = KN / dist ** 2 * (1 / NEGATIVE_BORDER - 1 / dist)
                forcex = forcex + force * math.cos(angle)
                forcey = forcey + force * math.sin(angle)

        angle = angle + 10 * scan.angle_increment  # increament angle for next group

    return forcex, forcey


def positive_force(dif_x, dif_y):  # Compute postive force
    dist = distan(0, 0, dif_x, dif_y)
    if dist > POSITIVE_BORDER:
        force = POSITIVE_BORDER * KP
    else:
        force = dist * KP
    fx = dif_x / dist * force
    fy = dif_y / dist * force
    return fx, fy


def distan(x1, y1, x2, y2):  # Compute distance between points
    dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return dist

def controller(x, y):  # compute best veolcity, basing on inforamtion from artificial potential fields
    distance = distan(0, 0, x, y)  # compute distance
    global front
    # theta = math.atan2(y,x)
    v = x / distance * PV  # compute linear velocity
    w = y / distance * PW  # compute angular velocity
    if (x < 0):  # if linear velocity is lower than 0
        v = 0  # velocity is 0
        w = math.copysign(PW, w)  # angular veloicty is pw
    if v > MAX_V:  # if linear velocity is too big
        w = w * (MAX_V / v)  # angular velocity is proportionally lower
        v = MAX_V  # linear velocity is equal to max_v
    if w > MAX_W:  # if linear velocity is too big
        v = v * (MAX_W / w)  # angular velocity is proportionally lower
        w= MAX_W  # linear velocity is equal to max_v

    return v, w
