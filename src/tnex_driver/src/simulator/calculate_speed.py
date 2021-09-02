#!/usr/bin/env python

"""Calculate and publish ego vehicle speed"""

import rospy
import numpy as np
from std_msgs.msg import Float64
from sensors import gnss

rospy.init_node('sim_calc_ego_vehicle_speed')

pub = rospy.Publisher('vehicle_speed', Float64, queue_size=10)
freq = 2


def create():
    rate = rospy.Rate(freq)
    prev_ev_loc = None
    while not rospy.is_shutdown():
        ev_loc = gnss.get_ego_vehicle_location()
        if ev_loc is not None and prev_ev_loc is not None:
            ds = np.linalg.norm(np.array([ev_loc.x - prev_ev_loc.x, ev_loc.y - prev_ev_loc.y]))
            dt = 1 / freq
            speed = ds / dt
            pub.publish(speed)
        prev_ev_loc = ev_loc
        rate.sleep()

def destroy():
    pass

try:
    rospy.on_shutdown(destroy)
    create()
except Exception as e:
    rospy.logerr('Error occured: ' + str(e))
