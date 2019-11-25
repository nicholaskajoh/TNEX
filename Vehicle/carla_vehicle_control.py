#!/usr/bin/env python

import glob
import os
import sys

try:
    sys.path.append(glob.glob('%s/dist/carla-*%d.%d-%s.egg' % (
        os.path.dirname(os.path.realpath(__file__)),
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import redis
import numpy as np
import time
import pickle


try:
    r = redis.Redis()
except redis.ConnectionError as e:
    print(e)

def apply_control(vehicle, message):
    if type(message['data']) is not int:
        control_params = pickle.loads(message['data'])
        throttle = control_params['throttle']
        steer = control_params['steer']
        brake = control_params['brake']
        reverse = control_params['reverse']
        control = carla.VehicleControl(throttle=throttle, steer=steer, brake=brake, reverse=reverse)
        vehicle.apply_control(control)

def carla_vehicle_control():
    # create simulator client
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    # get simulator world
    world = client.get_world()

    # get vehicle
    vehicle = None
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == 'autonomous_car':
            vehicle = actor

    if vehicle is None:
        raise Exception('Could not find vehicle')

    carla_control = r.pubsub()
    carla_control.subscribe('carla_control')

    print('Waiting for vehicle commands')

    try:
        while True:
            carla_control_message = carla_control.get_message()
            if carla_control_message:
                apply_control(vehicle, carla_control_message)

            time.sleep(0.03)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    carla_vehicle_control()
