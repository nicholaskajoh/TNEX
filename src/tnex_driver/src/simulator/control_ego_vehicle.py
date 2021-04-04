#!/usr/bin/env python

'''
Apply vehicle commands sent to vehicle_control topic.
'''

import glob
import sys
import os
try:
    sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import rospy
import time
from tnex_driver.msg import VehicleControl

rospy.init_node('sim_control_ego_vehicle')

ego_vehicle = None

def apply_control(data):
    global ego_vehicle
    
    reverse = True if data.reverse == 1 else False
    control = carla.VehicleControl(throttle=data.throttle, steer=data.steer, brake=data.brake, reverse=reverse)
    ego_vehicle.apply_control(control)

def main():
    global ego_vehicle

    # create simulator client
    param_host = rospy.get_param('host')
    param_port = int(rospy.get_param('port'))
    client = carla.Client(param_host, param_port)
    client.set_timeout(15.0)

    # get simulator world
    world = client.get_world()

    # get vehicle
    time.sleep(5) # wait for vehicle to spawn
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == 'ego_vehicle':
            ego_vehicle = actor

    if ego_vehicle is None:
        raise Exception('Could not find ego vehicle')

    rospy.Subscriber('vehicle_control', VehicleControl, apply_control)
    rospy.spin()

try:
    main()
except Exception as e:
    rospy.logerr('Error occured: ' + str(e))